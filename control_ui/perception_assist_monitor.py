"""Monitor-only morphology/load consistency for the WA100 hand.

The monitor consumes measured actuator position and filtered motor current.  It
never writes a command.  Motor current is deliberately described as a load
proxy: residuals are only valid when an explicitly commissioned no-contact
baseline is present for every required channel.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path


STATES = ("FREE", "EARLY_CONTACT", "GRASP_CANDIDATE", "GRASP_SUPPORTED", "OVERLOAD")


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def load_monitor_config(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


class PerceptionAssistMonitor:
    """Read-only WA100 grasp evidence state machine.

    ``update`` accepts a ``wa100_recording_state`` payload and returns a unified
    JSON-serialisable state message.  All timing is based on backend monotonic
    receive time; source UTC is retained for recording/alignment only.
    """

    def __init__(self, config):
        self.config = dict(config or {})
        self.channel_names = list(self.config.get("channel_names") or [])
        self.channel_count = len(self.channel_names)
        if not self.channel_count:
            raise ValueError("perception monitor requires channel_names")
        self.finger_names = list(self.config.get("finger_names") or self.channel_names)
        if len(self.finger_names) != self.channel_count:
            raise ValueError(f"finger_names length {len(self.finger_names)} != {self.channel_count}")
        self.min_grasp_loaded_fingers = int(
            self.config.get("min_grasp_loaded_fingers", self.config.get("min_grasp_loaded_channels", 2))
        )
        self.open_units = self._array("closure_open_units", None)
        self.close_units = self._array("closure_close_units", None)
        self.weights = [float(v) for v in self._array("closure_weights", 1.0)]
        weight_sum = sum(max(0.0, value) for value in self.weights)
        if weight_sum <= 0:
            raise ValueError("closure_weights must contain a positive weight")
        self.weights = [max(0.0, value) / weight_sum for value in self.weights]
        self.baselines = self._array("current_baselines", None)
        self.load_on = [float(v) for v in self._array("load_residual_on_ma", 0.0)]
        self.load_off = [float(v) for v in self._array("load_residual_off_ma", 0.0)]
        self.overload_on = [float(v) for v in self._array("overload_residual_on_ma", math.inf)]
        self.overload_off = [float(v) for v in self._array("overload_residual_off_ma", math.inf)]
        for index in range(self.channel_count):
            if self.load_off[index] > self.load_on[index]:
                raise ValueError("load residual off threshold must not exceed on threshold")
            if self.overload_off[index] > self.overload_on[index]:
                raise ValueError("overload residual off threshold must not exceed on threshold")

        self.state = "FREE"
        self.decision_seq = 0
        self._last_positions = None
        self._last_receive_monotonic_ns = None
        self._last_position_valid_monotonic_ns = [None] * self.channel_count
        self._loaded = [False] * self.channel_count
        self._candidate_since_ns = None
        self._overload_since_ns = None
        self._overload_clear_since_ns = None
        self._stable_samples = deque()
        self._last_valid = False

    def _array(self, key, default):
        value = self.config.get(key)
        if value is None:
            return [default] * self.channel_count
        value = list(value)
        if len(value) != self.channel_count:
            raise ValueError(f"{key} length {len(value)} != {self.channel_count}")
        return value

    def _invalid(self, payload, source_ns, receive_utc_ns, reasons, closure_score=None, residuals=None,
                 signal_validity=None, degraded_reasons=None):
        previous = self.state
        self.state = "FREE"
        self._loaded = [False] * self.channel_count
        self._candidate_since_ns = None
        self._overload_since_ns = None
        self._overload_clear_since_ns = None
        self._stable_samples.clear()
        events = []
        if self._last_valid or previous != "FREE":
            events.append("MONITOR_INVALID")
        self._last_valid = False
        return self._message(
            payload, source_ns, receive_utc_ns, False, reasons, closure_score,
            residuals or [None] * self.channel_count, [],
            "monitor_invalid_no_grasp_inference", events, signal_validity, degraded_reasons,
        )

    def _message(self, payload, source_ns, receive_utc_ns, valid, invalid_reasons,
                 closure_score, residuals, loaded_fingers, reason, events,
                 signal_validity=None, degraded_reasons=None):
        self.decision_seq += 1
        event = events[-1] if events else None
        prompts = {
            "EARLY_CONTACT": "边缘/障碍早碰：停止继续闭合并调整接近。",
            "GRASP_CANDIDATE": "疑似包覆抓持：请观察并确认，尚不能视为抓持成功。",
            "GRASP_SUPPORTED": "形态与多指负载短时稳定：请操作者确认保持，尚未证明抓持成功。",
            "OVERLOAD": "负载代理残差达到风险阈值：请松开或调整；WA100 本地保护仍独立生效。",
            "MONITOR_INVALID": "抓持辅助判别无效：仅保留原有控制与本地保护。",
        }
        timestamp_ns = int(source_ns or receive_utc_ns)
        return {
            "type": "perception_assist_state",
            "schema_version": 1,
            "timestamp": timestamp_ns / 1e9,
            "timestamp_ns": timestamp_ns,
            "timestamp_source": "wa100_source_time_ns" if source_ns else "bridge_receive_utc_ns",
            "receive_utc_ns": int(receive_utc_ns),
            "decision_seq": self.decision_seq,
            "frame_id": int(payload.get("seq") or 0),
            "state": self.state,
            "closure_score": closure_score,
            "filtered_current_ma": payload.get("filtered_current_ma"),
            "current_residual": residuals,
            "loaded_fingers": loaded_fingers,
            "valid": 1 if valid else 0,
            "invalid_reason": ";".join(dict.fromkeys(invalid_reasons)),
            "degraded_reason": ";".join(dict.fromkeys(degraded_reasons or [])),
            "signal_validity": signal_validity or {},
            "reason": reason,
            "events": events,
            "prompt_event": event,
            "prompt_message": prompts.get(event, ""),
            "monitor_only": True,
            "grasp_success_confirmed": False,
            "ablation": {
                "current_only": len(loaded_fingers) >= self.min_grasp_loaded_fingers,
                "closure_only": closure_score is not None and closure_score >= float(self.config.get("grasp_closure_enter", 0.6)),
                "joint": (
                    closure_score is not None
                    and closure_score >= float(self.config.get("grasp_closure_enter", 0.6))
                    and len(loaded_fingers) >= self.min_grasp_loaded_fingers
                ),
            },
        }

    def _closure(self, positions):
        reasons, closures = [], []
        for index, position in enumerate(positions):
            opened, closed = self.open_units[index], self.close_units[index]
            if not _finite(opened) or not _finite(closed) or abs(float(closed) - float(opened)) < 1e-9:
                reasons.append(f"missing_closure_calibration:{self.channel_names[index]}")
                closures.append(None)
                continue
            value = (float(position) - float(opened)) / (float(closed) - float(opened))
            closures.append(max(0.0, min(1.0, value)))
        if reasons:
            return None, closures, reasons
        return sum(weight * value for weight, value in zip(self.weights, closures)), closures, []

    def _baseline(self, index, position, velocity, direction):
        model = self.baselines[index]
        if not isinstance(model, dict) or not model.get("valid"):
            return None, f"missing_current_baseline:{self.channel_names[index]}"
        if model.get("type") != "affine_q_dq_direction":
            return None, f"unsupported_current_baseline:{self.channel_names[index]}"
        required = ("intercept_ma", "position_coeff_ma_per_unit", "speed_coeff_ma_per_unit_s")
        if any(not _finite(model.get(key)) for key in required):
            return None, f"invalid_current_baseline:{self.channel_names[index]}"
        offsets = model.get("direction_offset_ma") or {}
        offset = offsets.get(direction)
        if not _finite(offset):
            return None, f"missing_direction_baseline:{self.channel_names[index]}:{direction}"
        speed = abs(float(velocity))
        speed_clip = model.get("speed_clip_units_s")
        if _finite(speed_clip) and float(speed_clip) > 0:
            speed = min(speed, float(speed_clip))
        value = (
            float(model["intercept_ma"])
            + float(model["position_coeff_ma_per_unit"]) * float(position)
            + float(model["speed_coeff_ma_per_unit_s"]) * speed
            + float(offset)
        )
        floor = model.get("baseline_floor_ma")
        ceiling = model.get("baseline_ceiling_ma")
        if _finite(floor):
            value = max(value, float(floor))
        if _finite(ceiling):
            value = min(value, float(ceiling))
        return value, None

    def _stable(self, now_ns, _loaded_indices):
        window_ns = int(float(self.config.get("supported_stability_window_sec", 0.30)) * 1e9)
        hold_ns = int(float(self.config.get("supported_hold_sec", 0.30)) * 1e9)
        cutoff = now_ns - window_ns
        while self._stable_samples and self._stable_samples[0][0] < cutoff:
            self._stable_samples.popleft()
        if len(self._stable_samples) < 2 or self._stable_samples[-1][0] - self._stable_samples[0][0] < hold_ns:
            return False
        if max(item[1] for item in self._stable_samples) - min(item[1] for item in self._stable_samples) > float(self.config.get("closure_stability_range", 0.06)):
            return False
        minimum = self.min_grasp_loaded_fingers
        loaded_sets = [set(item[3]) for item in self._stable_samples]
        common_loaded = set.intersection(*loaded_sets) if loaded_sets else set()
        common_fingers = {self.finger_names[index] for index in common_loaded}
        if len(common_fingers) < minimum:
            return False
        residual_range = float(self.config.get("residual_stability_range_ma", 100.0))
        for index in common_loaded:
            values = [item[2][index] for item in self._stable_samples]
            if max(values) - min(values) > residual_range:
                return False
        return True

    def update(self, payload, receive_utc_ns=None, receive_monotonic_ns=None):
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        receive_monotonic_ns = int(receive_monotonic_ns or time.monotonic_ns())
        source_ns = int(payload.get("source_time_ns") or 0)
        reasons = []
        if not self.config.get("enabled", True):
            return self._invalid(payload, source_ns, receive_utc_ns, ["monitor_disabled"])

        positions = payload.get("actual_position_units")
        currents = payload.get("filtered_current_ma")
        frame_valid = bool(payload.get("feedback_valid"))
        positions_shape_valid = (
            isinstance(positions, (list, tuple))
            and len(positions) == self.channel_count
            and all(_finite(v) for v in positions)
        )
        currents_shape_valid = (
            isinstance(currents, (list, tuple))
            and len(currents) == self.channel_count
            and all(_finite(v) for v in currents)
        )
        if not positions_shape_valid:
            reasons.append("actual_position_missing_or_invalid")
        if not currents_shape_valid:
            reasons.append("filtered_current_missing_or_invalid")

        position_valid = payload.get("position_valid")
        if position_valid is None:
            position_valid = [frame_valid] * self.channel_count
        elif not isinstance(position_valid, (list, tuple)) or len(position_valid) != self.channel_count:
            reasons.append("position_valid_mask_invalid")
            position_valid = [False] * self.channel_count
        else:
            position_valid = [bool(value) for value in position_valid]

        current_valid = payload.get("current_valid")
        if current_valid is None:
            current_valid = [frame_valid] * self.channel_count
        elif not isinstance(current_valid, (list, tuple)) or len(current_valid) != self.channel_count:
            reasons.append("current_valid_mask_invalid")
            current_valid = [False] * self.channel_count
        else:
            current_valid = [bool(value) for value in current_valid]

        if not frame_valid:
            reasons.append("wa100_feedback_invalid")
        if currents_shape_valid:
            reasons.extend(
                f"filtered_current_channel_invalid:{self.channel_names[index]}"
                for index, valid in enumerate(current_valid) if not valid
            )

        fallback_channels = []
        degraded_reasons = []
        zero_wrap_corrected = payload.get("position_zero_wrap_corrected")
        if not isinstance(zero_wrap_corrected, (list, tuple)) or len(zero_wrap_corrected) != self.channel_count:
            zero_wrap_corrected = [False] * self.channel_count
        else:
            zero_wrap_corrected = [bool(value) for value in zero_wrap_corrected]
        corrected_channels = [index for index, corrected in enumerate(zero_wrap_corrected) if corrected]
        degraded_reasons.extend(
            f"position_zero_wrap_corrected:{self.channel_names[index]}" for index in corrected_channels
        )
        fallback_grace_ns = int(float(self.config.get("position_fallback_grace_sec", 0.25)) * 1e9)
        if positions_shape_valid:
            for index, valid in enumerate(position_valid):
                if valid:
                    self._last_position_valid_monotonic_ns[index] = receive_monotonic_ns
                    continue
                last_valid_ns = self._last_position_valid_monotonic_ns[index]
                if last_valid_ns is not None and receive_monotonic_ns - last_valid_ns <= fallback_grace_ns:
                    fallback_channels.append(index)
                    degraded_reasons.append(f"position_fallback:{self.channel_names[index]}")
                else:
                    reasons.append(f"actual_position_channel_invalid:{self.channel_names[index]}")

        stale_ns = int(float(self.config.get("stale_timeout_sec", 0.25)) * 1e9)
        if source_ns and (receive_utc_ns - source_ns > stale_ns or source_ns - receive_utc_ns > stale_ns):
            reasons.append("data_stale")
        signal_validity = {
            "feedback_frame": frame_valid,
            "current": currents_shape_valid and all(current_valid),
            "position_current_frame": positions_shape_valid and all(position_valid),
            "position_effective": positions_shape_valid and not any(
                reason.startswith("actual_position_channel_invalid:") for reason in reasons
            ),
            "position_fallback_channels": [self.channel_names[index] for index in fallback_channels],
            "position_zero_wrap_corrected_channels": [self.channel_names[index] for index in corrected_channels],
        }
        if reasons:
            return self._invalid(
                payload, source_ns, receive_utc_ns, reasons,
                signal_validity=signal_validity, degraded_reasons=degraded_reasons,
            )

        positions = [float(value) for value in positions]
        currents = [float(value) for value in currents]
        if fallback_channels and self._last_positions is not None:
            for index in fallback_channels:
                positions[index] = self._last_positions[index]
        closure_score, _closures, closure_reasons = self._closure(positions)
        reasons.extend(closure_reasons)
        for index, model in enumerate(self.baselines):
            if not isinstance(model, dict) or not model.get("valid"):
                reasons.append(f"missing_current_baseline:{self.channel_names[index]}")

        velocities = None
        if self._last_positions is not None and self._last_receive_monotonic_ns is not None:
            dt = (receive_monotonic_ns - self._last_receive_monotonic_ns) / 1e9
            if 0.001 <= dt <= float(self.config.get("max_velocity_dt_sec", 0.20)):
                fallback_set = set(fallback_channels)
                velocities = [
                    0.0 if index in fallback_set else (position - previous) / dt
                    for index, (position, previous) in enumerate(zip(positions, self._last_positions))
                ]
            else:
                reasons.append("velocity_history_timing_invalid")
        else:
            reasons.append("insufficient_velocity_history")
        self._last_positions = positions
        self._last_receive_monotonic_ns = receive_monotonic_ns

        residuals = [None] * self.channel_count
        if velocities is not None:
            deadband = float(self.config.get("direction_deadband_units_s", 10.0))
            for index, (position, velocity, opened, closed) in enumerate(zip(positions, velocities, self.open_units, self.close_units)):
                if not _finite(opened) or not _finite(closed):
                    continue
                closure_velocity = velocity / (float(closed) - float(opened))
                direction = "closing" if closure_velocity > deadband / max(abs(float(closed) - float(opened)), 1.0) else (
                    "opening" if closure_velocity < -deadband / max(abs(float(closed) - float(opened)), 1.0) else "stationary"
                )
                baseline, baseline_reason = self._baseline(index, position, velocity, direction)
                if baseline_reason:
                    reasons.append(baseline_reason)
                else:
                    residuals[index] = currents[index] - baseline

        if reasons:
            return self._invalid(
                payload, source_ns, receive_utc_ns, reasons, closure_score, residuals,
                signal_validity, degraded_reasons,
            )

        loaded_indices = []
        for index, residual in enumerate(residuals):
            if self._loaded[index]:
                self._loaded[index] = residual >= self.load_off[index]
            else:
                self._loaded[index] = residual >= self.load_on[index]
            if self._loaded[index]:
                loaded_indices.append(index)
        loaded_names = list(dict.fromkeys(self.finger_names[index] for index in loaded_indices))

        overload_indices = [
            index for index, residual in enumerate(residuals)
            if residual >= (self.overload_off[index] if self.state == "OVERLOAD" else self.overload_on[index])
        ]
        overload_hold_ns = int(float(self.config.get("overload_hold_sec", 0.04)) * 1e9)
        overload_release_ns = int(float(self.config.get("overload_release_sec", 0.10)) * 1e9)
        if overload_indices:
            self._overload_clear_since_ns = None
            if self._overload_since_ns is None:
                self._overload_since_ns = receive_monotonic_ns
        else:
            self._overload_since_ns = None
            if self.state == "OVERLOAD" and self._overload_clear_since_ns is None:
                self._overload_clear_since_ns = receive_monotonic_ns

        previous = self.state
        reason = "no_joint_closure_load_evidence"
        if self.state == "OVERLOAD" and (
            overload_indices or self._overload_clear_since_ns is None
            or receive_monotonic_ns - self._overload_clear_since_ns < overload_release_ns
        ):
            self.state = "OVERLOAD"
            reason = "load_proxy_residual_overload_risk"
        elif overload_indices and receive_monotonic_ns - self._overload_since_ns >= overload_hold_ns:
            self.state = "OVERLOAD"
            reason = "load_proxy_residual_overload_risk"
        else:
            if previous == "OVERLOAD":
                self._overload_clear_since_ns = None
            enter = float(self.config.get("grasp_closure_enter", 0.62))
            exit_threshold = float(self.config.get("grasp_closure_exit", 0.55))
            closure_threshold = exit_threshold if previous in {"GRASP_CANDIDATE", "GRASP_SUPPORTED"} else enter
            minimum_loaded = self.min_grasp_loaded_fingers
            grasp_condition = closure_score >= closure_threshold and len(loaded_names) >= minimum_loaded
            if grasp_condition:
                if self._candidate_since_ns is None:
                    self._candidate_since_ns = receive_monotonic_ns
                self._stable_samples.append((receive_monotonic_ns, closure_score, tuple(residuals), tuple(loaded_indices)))
                candidate_hold_ns = int(float(self.config.get("candidate_hold_sec", 0.12)) * 1e9)
                if receive_monotonic_ns - self._candidate_since_ns >= candidate_hold_ns:
                    self.state = "GRASP_SUPPORTED" if self._stable(receive_monotonic_ns, loaded_indices) else "GRASP_CANDIDATE"
                    reason = "stable_multichannel_morphology_load_consistency" if self.state == "GRASP_SUPPORTED" else "sustained_morphology_load_consistency_candidate"
                else:
                    self.state = "FREE"
                    reason = "joint_evidence_dwell_pending"
            else:
                self._candidate_since_ns = None
                self._stable_samples.clear()
                if closure_score < enter and loaded_indices:
                    self.state = "EARLY_CONTACT"
                    reason = "load_proxy_before_grasp_closure_threshold"
                else:
                    self.state = "FREE"
                    reason = "no_joint_closure_load_evidence"

        event_for_state = {
            "EARLY_CONTACT": "EARLY_CONTACT",
            "GRASP_CANDIDATE": "GRASP_CANDIDATE",
            "GRASP_SUPPORTED": "GRASP_SUPPORTED",
            "OVERLOAD": "OVERLOAD",
        }
        events = [event_for_state[self.state]] if self.state != previous and self.state in event_for_state else []
        self._last_valid = True
        return self._message(
            payload, source_ns, receive_utc_ns, True, [], closure_score, residuals,
            loaded_names, reason, events, signal_validity, degraded_reasons,
        )
