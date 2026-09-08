"""Causal, fixed-rate planning for Canonical wrist commands.

The planner owns the published wrist state.  Adapters remain responsible for
mapping device coordinates into Canonical coordinates, while this module makes
the resulting command continuous and records the raw/planned boundary.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Any, Mapping, Optional, Sequence

import numpy as np


PLANNER_NAMES = ("off", "jerk_limited", "one_euro")


def _finite_vector(value: Any, size: int) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size != size or not np.all(np.isfinite(result)):
        raise ValueError(f"expected {size} finite values")
    return result


def _normalize_quaternion(value: Any) -> np.ndarray:
    quaternion = _finite_vector(value, 4)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    quaternion /= norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def _roll_from_quaternion(value: Any) -> float:
    x, y, z, w = _normalize_quaternion(value)
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def _roll_quaternion(roll: float) -> list[float]:
    half = float(roll) * 0.5
    quaternion = np.asarray([math.sin(half), 0.0, 0.0, math.cos(half)], dtype=float)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion.tolist()


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _closest_angle(reference: float, value: float) -> float:
    return float(reference) + _wrap_angle(float(value) - float(reference))


@dataclass(frozen=True)
class CanonicalPlannerConfig:
    enabled: bool = True
    publish_hz: float = 50.0
    keyboard_profile: str = "jerk_limited"
    gamepad_profile: str = "jerk_limited"
    pico_wrist_profile: str = "one_euro"
    vr_controller_profile: str = "one_euro"
    translation_max_velocity_m_s: float = 0.20
    translation_max_acceleration_m_s2: float = 1.0
    translation_max_jerk_m_s3: float = 10.0
    roll_max_velocity_rad_s: float = 1.0
    roll_max_acceleration_rad_s2: float = 5.0
    roll_max_jerk_rad_s3: float = 50.0
    one_euro_min_cutoff_hz: float = 2.0
    one_euro_beta: float = 0.6
    one_euro_derivative_cutoff_hz: float = 1.0
    one_euro_max_velocity_m_s: float = 0.20
    one_euro_max_roll_velocity_rad_s: float = 1.5
    one_euro_innovation_base_m: float = 0.003
    one_euro_prediction_horizon_sec: float = 0.06
    one_euro_prediction_velocity_threshold_m_s: float = 0.03
    one_euro_prediction_full_velocity_m_s: float = 0.10
    one_euro_prediction_roll_threshold_rad_s: float = 0.10
    one_euro_prediction_full_roll_rad_s: float = 0.50
    hold_timeout_sec: float = 0.10
    vr_stability_enabled: bool = True
    vr_still_cutoff_hz: float = 0.4
    vr_translation_deadband_m: float = 0.0015
    vr_roll_deadband_rad: float = 0.004

    @classmethod
    def from_mapping(cls, values: Optional[Mapping[str, Any]], publish_hz: float = 50.0) -> "CanonicalPlannerConfig":
        values = values or {}

        def boolean(name: str, default: bool) -> bool:
            value = values.get(name, default)
            if isinstance(value, bool):
                return value
            return str(value).strip().lower() in {"1", "true", "yes", "on"}

        config = cls(
            enabled=boolean("planner_enabled", True),
            publish_hz=float(values.get("publish_hz", publish_hz)),
            keyboard_profile=str(values.get("planner_keyboard_profile", "jerk_limited")),
            gamepad_profile=str(values.get("planner_gamepad_profile", "jerk_limited")),
            pico_wrist_profile=str(values.get("planner_pico_wrist_profile", "one_euro")),
            vr_controller_profile=str(values.get("planner_vr_controller_profile", "one_euro")),
            translation_max_velocity_m_s=float(values.get("planner_translation_max_velocity_m_s", 0.20)),
            translation_max_acceleration_m_s2=float(values.get("planner_translation_max_acceleration_m_s2", 1.0)),
            translation_max_jerk_m_s3=float(values.get("planner_translation_max_jerk_m_s3", 10.0)),
            roll_max_velocity_rad_s=float(values.get("planner_roll_max_velocity_rad_s", 1.0)),
            roll_max_acceleration_rad_s2=float(values.get("planner_roll_max_acceleration_rad_s2", 5.0)),
            roll_max_jerk_rad_s3=float(values.get("planner_roll_max_jerk_rad_s3", 50.0)),
            one_euro_min_cutoff_hz=float(values.get("planner_one_euro_min_cutoff_hz", 2.0)),
            one_euro_beta=float(values.get("planner_one_euro_beta", 0.6)),
            one_euro_derivative_cutoff_hz=float(values.get("planner_one_euro_derivative_cutoff_hz", 1.0)),
            one_euro_max_velocity_m_s=float(values.get("planner_one_euro_max_velocity_m_s", 0.20)),
            one_euro_max_roll_velocity_rad_s=float(values.get("planner_one_euro_max_roll_velocity_rad_s", 1.5)),
            one_euro_innovation_base_m=float(values.get("planner_one_euro_innovation_base_m", 0.003)),
            one_euro_prediction_horizon_sec=float(values.get("planner_one_euro_prediction_horizon_sec", 0.06)),
            one_euro_prediction_velocity_threshold_m_s=float(values.get("planner_one_euro_prediction_velocity_threshold_m_s", 0.03)),
            one_euro_prediction_full_velocity_m_s=float(values.get("planner_one_euro_prediction_full_velocity_m_s", 0.10)),
            one_euro_prediction_roll_threshold_rad_s=float(values.get("planner_one_euro_prediction_roll_threshold_rad_s", 0.10)),
            one_euro_prediction_full_roll_rad_s=float(values.get("planner_one_euro_prediction_full_roll_rad_s", 0.50)),
            hold_timeout_sec=float(values.get("planner_hold_timeout_sec", 0.10)),
            vr_stability_enabled=boolean("planner_vr_stability_enabled", True),
            vr_still_cutoff_hz=float(values.get("planner_vr_still_cutoff_hz", 0.4)),
            vr_translation_deadband_m=float(values.get("planner_vr_translation_deadband_m", 0.0015)),
            vr_roll_deadband_rad=float(values.get("planner_vr_roll_deadband_rad", 0.004)),
        )
        config.validate()
        return config

    def validate(self) -> None:
        profiles = (
            self.keyboard_profile,
            self.gamepad_profile,
            self.pico_wrist_profile,
            self.vr_controller_profile,
        )
        if any(profile not in PLANNER_NAMES for profile in profiles):
            raise ValueError(f"planner profiles must be one of {PLANNER_NAMES}")
        positive = (
            self.vr_still_cutoff_hz,
            self.vr_translation_deadband_m,
            self.vr_roll_deadband_rad,
            self.publish_hz,
            self.translation_max_velocity_m_s,
            self.translation_max_acceleration_m_s2,
            self.translation_max_jerk_m_s3,
            self.roll_max_velocity_rad_s,
            self.roll_max_acceleration_rad_s2,
            self.roll_max_jerk_rad_s3,
            self.one_euro_min_cutoff_hz,
            self.one_euro_derivative_cutoff_hz,
            self.one_euro_max_velocity_m_s,
            self.one_euro_max_roll_velocity_rad_s,
            self.one_euro_prediction_horizon_sec,
            self.one_euro_prediction_full_velocity_m_s,
            self.one_euro_prediction_full_roll_rad_s,
            self.hold_timeout_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("planner limits and rates must be positive finite values")
        if not math.isfinite(self.one_euro_beta) or self.one_euro_beta < 0.0:
            raise ValueError("One Euro beta must be finite and non-negative")
        if not math.isfinite(self.one_euro_innovation_base_m) or self.one_euro_innovation_base_m < 0.0:
            raise ValueError("One Euro innovation base must be finite and non-negative")
        if (
            not math.isfinite(self.one_euro_prediction_velocity_threshold_m_s)
            or not math.isfinite(self.one_euro_prediction_roll_threshold_rad_s)
            or self.one_euro_prediction_velocity_threshold_m_s < 0.0
            or self.one_euro_prediction_roll_threshold_rad_s < 0.0
            or self.one_euro_prediction_full_velocity_m_s <= self.one_euro_prediction_velocity_threshold_m_s
            or self.one_euro_prediction_full_roll_rad_s <= self.one_euro_prediction_roll_threshold_rad_s
        ):
            raise ValueError("One Euro prediction thresholds are invalid")

    def profile_for(self, source: str) -> str:
        if not self.enabled:
            return "off"
        return {
            "keyboard": self.keyboard_profile,
            "gamepad": self.gamepad_profile,
            "pico_wrist": self.pico_wrist_profile,
            "vr_controller": self.vr_controller_profile,
        }.get(str(source), "off")

    @property
    def config_id(self) -> str:
        return "canonical-wrist-planner-v1"

    @property
    def content_sha256(self) -> str:
        payload = json.dumps(asdict(self), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()


class _OneEuroVector:
    def __init__(self, size: int, min_cutoff_hz: float, beta: float, derivative_cutoff_hz: float):
        self.size = int(size)
        self.min_cutoff_hz = float(min_cutoff_hz)
        self.beta = float(beta)
        self.derivative_cutoff_hz = float(derivative_cutoff_hz)
        self.filtered: Optional[np.ndarray] = None
        self.raw: Optional[np.ndarray] = None
        self.derivative: Optional[np.ndarray] = None
        self.still = [True, True]
        self.motion_evidence = [0.0, 0.0]

    @staticmethod
    def _alpha(cutoff_hz: np.ndarray | float, dt: float) -> np.ndarray:
        cutoff = np.asarray(cutoff_hz, dtype=float)
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / float(dt))

    def reset(self, value: Sequence[float]) -> None:
        vector = _finite_vector(value, self.size)
        self.filtered = vector.copy()
        self.raw = vector.copy()
        self.derivative = np.zeros(self.size, dtype=float)
        self.still = [True, True]
        self.motion_evidence = [0.0, 0.0]

    def update(self, value: Sequence[float], dt: float,
               stability: Optional[CanonicalPlannerConfig] = None) -> np.ndarray:
        vector = _finite_vector(value, self.size)
        if self.filtered is None or self.raw is None or self.derivative is None:
            self.reset(vector)
            return vector.copy()
        derivative = (vector - self.raw) / dt
        derivative_alpha = self._alpha(self.derivative_cutoff_hz, dt)
        self.derivative += derivative_alpha * (derivative - self.derivative)
        cutoff = self.min_cutoff_hz + self.beta * np.abs(self.derivative)
        target = vector.copy()
        if stability is not None:
            for group, indices, deadband in (
                (0, slice(0, 3), stability.vr_translation_deadband_m),
                (1, slice(3, 4), stability.vr_roll_deadband_rad),
            ):
                error = target[indices] - self.filtered[indices]
                distance = float(np.linalg.norm(error))
                speed = float(np.linalg.norm(self.derivative[indices]))
                if self.still[group]:
                    moving = distance >= 4.0 * deadband or speed >= 20.0 * deadband
                    self.motion_evidence[group] = self.motion_evidence[group] + dt if moving else 0.0
                    if self.motion_evidence[group] >= 0.04:
                        self.still[group] = False
                else:
                    self.still[group] = distance < 2.0 * deadband and speed < 5.0 * deadband
                    self.motion_evidence[group] = 0.0
                if self.still[group]:
                    cutoff[indices] = stability.vr_still_cutoff_hz
                    # Continuous soft deadzone; accumulated displacement can
                    # always release the hold, including very slow motion.
                    target[indices] = self.filtered[indices] + error * max(0.0, 1.0 - deadband / max(distance, 1e-12))
        alpha = self._alpha(cutoff, dt)
        self.filtered += alpha * (target - self.filtered)
        self.raw = vector.copy()
        return self.filtered.copy()


class CanonicalWristPlanner:
    """Source-adaptive planner producing the authoritative Canonical pose."""

    def __init__(self, config: CanonicalPlannerConfig):
        self.config = config
        self.override_profile: Optional[str] = None
        self.source = ""
        self.profile = "off"
        self.last_time_ns: Optional[int] = None
        self.last_source_receive_ns: Optional[int] = None
        self.position: Optional[np.ndarray] = None
        self.roll = 0.0
        self.velocity = np.zeros(4, dtype=float)
        self.acceleration = np.zeros(4, dtype=float)
        self.last_measurement: Optional[np.ndarray] = None
        self.one_euro = _OneEuroVector(
            4,
            config.one_euro_min_cutoff_hz,
            config.one_euro_beta,
            config.one_euro_derivative_cutoff_hz,
        )
        self.last_reset_reason = "uninitialized"
        self.last_metadata: dict[str, Any] = {}

    def set_override(self, profile: Optional[str]) -> str:
        normalized = None if profile in (None, "", "default", "auto") else str(profile)
        if normalized is not None and normalized not in PLANNER_NAMES:
            raise ValueError(f"planner override must be default or one of {PLANNER_NAMES}")
        if normalized != self.override_profile:
            self.override_profile = normalized
            self.source = ""
            self.last_reset_reason = "profile_override_changed"
        return self.override_profile or "default"

    def _selected_profile(self, source: str) -> str:
        return self.override_profile or self.config.profile_for(source)

    def reset_tracking_reference(self) -> None:
        if self.position is not None:
            self._reset_state(self.position, self.roll, "tracking_reference_changed")
        self.last_time_ns = None

    def _pose(self) -> dict[str, list[float]]:
        if self.position is None:
            raise RuntimeError("planner is not initialized")
        return {
            "position_m": self.position.tolist(),
            "orientation_xyzw": _roll_quaternion(self.roll),
        }

    def _reset_state(self, position: np.ndarray, roll: float, reason: str) -> None:
        self.position = position.copy()
        self.roll = float(roll)
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)
        self.last_measurement = np.r_[position, roll]
        self.one_euro.reset(self.last_measurement)
        self.last_reset_reason = str(reason)

    @staticmethod
    def _axis_jerk_step(position: float, velocity: float, acceleration: float, target: float,
                        dt: float, max_velocity: float, max_acceleration: float,
                        max_jerk: float) -> tuple[float, float, float]:
        error = float(target) - float(position)
        if abs(error) < 1e-9:
            next_acceleration = acceleration + float(np.clip(
                -acceleration, -max_jerk * dt, max_jerk * dt
            ))
            if abs(next_acceleration) < 1e-9:
                next_acceleration = 0.0
            return float(target), 0.0, float(next_acceleration)
        stopping_velocity = math.sqrt(max(0.0, 2.0 * max_acceleration * abs(error)))
        desired_velocity = math.copysign(min(max_velocity, stopping_velocity), error) if error else 0.0
        desired_acceleration = float(np.clip((desired_velocity - velocity) / dt, -max_acceleration, max_acceleration))
        next_acceleration = acceleration + float(np.clip(
            desired_acceleration - acceleration, -max_jerk * dt, max_jerk * dt
        ))
        next_velocity = float(np.clip(velocity + next_acceleration * dt, -max_velocity, max_velocity))
        next_position = position + next_velocity * dt
        if error * (float(target) - next_position) <= 0.0:
            next_position = float(target)
            next_velocity = 0.0
            next_acceleration = acceleration + float(np.clip(
                -acceleration, -max_jerk * dt, max_jerk * dt
            ))
        return float(next_position), next_velocity, float(next_acceleration)

    def _jerk_limited_step(self, target_position: np.ndarray, target_roll: float, dt: float) -> None:
        target = np.r_[target_position, target_roll]
        current = np.r_[self.position, self.roll]
        maximum_velocity = np.r_[np.full(3, self.config.translation_max_velocity_m_s), self.config.roll_max_velocity_rad_s]
        maximum_acceleration = np.r_[np.full(3, self.config.translation_max_acceleration_m_s2), self.config.roll_max_acceleration_rad_s2]
        maximum_jerk = np.r_[np.full(3, self.config.translation_max_jerk_m_s3), self.config.roll_max_jerk_rad_s3]
        for axis in range(4):
            current[axis], self.velocity[axis], self.acceleration[axis] = self._axis_jerk_step(
                current[axis], self.velocity[axis], self.acceleration[axis], target[axis], dt,
                maximum_velocity[axis], maximum_acceleration[axis], maximum_jerk[axis],
            )
        self.position = current[:3]
        self.roll = float(current[3])

    def _one_euro_step(self, target_position: np.ndarray, target_roll: float, dt: float,
                       stabilize: bool = False) -> None:
        measurement = np.r_[target_position, target_roll]
        if self.last_measurement is None:
            self.last_measurement = measurement.copy()
        innovation_limit = self.config.one_euro_innovation_base_m + self.config.one_euro_max_velocity_m_s * dt
        translation_innovation = measurement[:3] - self.last_measurement[:3]
        translation_norm = float(np.linalg.norm(translation_innovation))
        if translation_norm > innovation_limit:
            measurement[:3] = self.last_measurement[:3] + translation_innovation * (innovation_limit / translation_norm)
        roll_limit = self.config.one_euro_max_roll_velocity_rad_s * dt
        measurement[3] = self.last_measurement[3] + float(np.clip(
            measurement[3] - self.last_measurement[3], -roll_limit, roll_limit
        ))
        self.last_measurement = measurement.copy()
        filtered = self.one_euro.update(measurement, dt, self.config if stabilize else None)
        derivative = np.asarray(self.one_euro.derivative, dtype=float)
        translation_speed = float(np.linalg.norm(derivative[:3]))
        translation_prediction_ratio = float(np.clip(
            (translation_speed - self.config.one_euro_prediction_velocity_threshold_m_s) /
            (self.config.one_euro_prediction_full_velocity_m_s -
             self.config.one_euro_prediction_velocity_threshold_m_s),
            0.0, 1.0,
        ))
        roll_speed = abs(float(derivative[3]))
        roll_prediction_ratio = float(np.clip(
            (roll_speed - self.config.one_euro_prediction_roll_threshold_rad_s) /
            (self.config.one_euro_prediction_full_roll_rad_s -
             self.config.one_euro_prediction_roll_threshold_rad_s),
            0.0, 1.0,
        ))
        if stabilize:
            if self.one_euro.still[0]:
                translation_prediction_ratio = 0.0
            if self.one_euro.still[1]:
                roll_prediction_ratio = 0.0
        predicted = filtered.copy()
        predicted[:3] += (
            derivative[:3] * self.config.one_euro_prediction_horizon_sec *
            translation_prediction_ratio
        )
        predicted[3] += (
            derivative[3] * self.config.one_euro_prediction_horizon_sec *
            roll_prediction_ratio
        )
        # Prediction may cancel filter phase lag but must not lead past the
        # current measured target on any component.
        filtered = np.clip(predicted, np.minimum(filtered, measurement), np.maximum(filtered, measurement))
        previous = np.r_[self.position, self.roll]
        translation_step = filtered[:3] - previous[:3]
        maximum_step = self.config.one_euro_max_velocity_m_s * dt
        step_norm = float(np.linalg.norm(translation_step))
        if step_norm > maximum_step:
            filtered[:3] = previous[:3] + translation_step * (maximum_step / step_norm)
        filtered[3] = previous[3] + float(np.clip(
            filtered[3] - previous[3],
            -self.config.one_euro_max_roll_velocity_rad_s * dt,
            self.config.one_euro_max_roll_velocity_rad_s * dt,
        ))
        self.velocity = (filtered - previous) / dt
        self.acceleration.fill(0.0)
        self.position = filtered[:3]
        self.roll = float(filtered[3])

    def step(self, *, source: str, target_pose: Mapping[str, Any], source_receive_ns: int,
             now_ns: Optional[int] = None, valid: bool = True) -> tuple[dict[str, list[float]], list[float], dict[str, Any]]:
        started_ns = time.perf_counter_ns()
        now_ns = int(now_ns or time.time_ns())
        source_receive_ns = int(source_receive_ns)
        target_position = _finite_vector(target_pose.get("position_m"), 3)
        target_roll = _roll_from_quaternion(target_pose.get("orientation_xyzw"))
        profile = self._selected_profile(source)
        nominal_dt = 1.0 / self.config.publish_hz
        dt = nominal_dt if self.last_time_ns is None else (now_ns - self.last_time_ns) / 1e9
        dt = float(np.clip(dt, nominal_dt * 0.25, nominal_dt * 5.0))
        source_age_sec = max(0.0, (now_ns - source_receive_ns) / 1e9)
        effective_valid = bool(valid and source_age_sec <= self.config.hold_timeout_sec)
        previous_pose = self._pose() if self.position is not None else None
        reset_reason = ""

        if self.position is None:
            self._reset_state(target_position, target_roll, "initialized")
            reset_reason = "initialized"
        elif source != self.source or profile != self.profile:
            if profile == "off":
                self._reset_state(target_position, target_roll, "source_or_profile_changed")
            else:
                self._reset_state(self.position, self.roll, "source_or_profile_changed")
            reset_reason = "source_or_profile_changed"
        elif not effective_valid:
            self.velocity.fill(0.0)
            self.acceleration.fill(0.0)
            if source == "pico_wrist" and self.config.vr_stability_enabled:
                self.one_euro.reset(np.r_[self.position, self.roll])
                self.last_measurement = np.r_[self.position, self.roll]
            reset_reason = "invalid_hold"
        else:
            target_roll = _closest_angle(self.roll, target_roll)
            if profile == "off":
                self._reset_state(target_position, target_roll, "off_passthrough")
            elif profile == "jerk_limited":
                self._jerk_limited_step(target_position, target_roll, dt)
            elif profile == "one_euro":
                self._one_euro_step(target_position, target_roll, dt,
                                    source == "pico_wrist" and self.config.vr_stability_enabled)
            else:  # Config validation should make this unreachable.
                raise ValueError(f"unsupported planner profile {profile}")

        self.source = str(source)
        self.profile = str(profile)
        self.last_time_ns = now_ns
        self.last_source_receive_ns = source_receive_ns
        output_pose = self._pose()
        if previous_pose is None or reset_reason == "source_or_profile_changed":
            delta = [0.0] * 6
        else:
            previous_position = np.asarray(previous_pose["position_m"], dtype=float)
            delta_position = self.position - previous_position
            previous_roll = _roll_from_quaternion(previous_pose["orientation_xyzw"])
            delta = [*delta_position.tolist(), _wrap_angle(self.roll - previous_roll), 0.0, 0.0]
        processing_time_ms = (time.perf_counter_ns() - started_ns) / 1e6
        hold_expired = not effective_valid
        metadata = {
            "name": profile,
            "profile": profile,
            "config_id": self.config.config_id,
            "config_sha256": self.config.content_sha256,
            "reset_reason": reset_reason,
            "processing_time_ms": processing_time_ms,
            "source_age_sec": source_age_sec,
            "hold": not effective_valid,
            "hold_expired": hold_expired,
            "valid_for_dispatch": bool(effective_valid),
            "override": self.override_profile or "default",
            "vr_stability": {
                "enabled": source == "pico_wrist" and profile == "one_euro" and self.config.vr_stability_enabled,
                "translation_still": self.one_euro.still[0],
                "roll_still": self.one_euro.still[1],
            },
        }
        self.last_metadata = metadata
        return output_pose, delta, dict(metadata)

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.config.enabled,
            "name": self.profile,
            "override": self.override_profile or "default",
            "config_id": self.config.config_id,
            "config_sha256": self.config.content_sha256,
            **dict(self.last_metadata),
        }
