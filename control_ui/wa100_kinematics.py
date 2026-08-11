#!/usr/bin/env python3
"""Reusable, ROS-independent WA100 hand forward kinematics.

The compiled model separates immutable URDF geometry from replaceable
motor-unit calibration.  Runtime consumers load one JSON file and can obtain
joint angles, link transforms, skeleton feature points and fingertip poses.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def _vector(value: Sequence[float] | str | None, default: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    if value is None:
        value = default
    if isinstance(value, str):
        value = [float(item) for item in value.split()]
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size != 3 or not np.all(np.isfinite(result)):
        raise ValueError("expected three finite values")
    return result


def rotation_axis(axis: Sequence[float], angle_rad: float) -> np.ndarray:
    x, y, z = _vector(axis)
    norm = math.sqrt(x*x + y*y + z*z)
    if norm <= 1e-12:
        raise ValueError("joint axis must be non-zero")
    x, y, z = x / norm, y / norm, z / norm
    c, s, k = math.cos(angle_rad), math.sin(angle_rad), 1.0 - math.cos(angle_rad)
    return np.array([
        [c + x*x*k, x*y*k - z*s, x*z*k + y*s],
        [y*x*k + z*s, c + y*y*k, y*z*k - x*s],
        [z*x*k - y*s, z*y*k + x*s, c + z*z*k],
    ], dtype=float)


def rotation_rpy(rpy_rad: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = _vector(rpy_rad)
    return rotation_axis((0, 0, 1), yaw) @ rotation_axis((0, 1, 0), pitch) @ rotation_axis((1, 0, 0), roll)


def transform(rotation: np.ndarray | None = None, translation: Sequence[float] | None = None) -> np.ndarray:
    result = np.eye(4, dtype=float)
    if rotation is not None:
        result[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    if translation is not None:
        result[:3, 3] = _vector(translation)
    return result


@dataclass(frozen=True)
class JointGeometry:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz_m: tuple[float, float, float]
    origin_rpy_rad: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower_rad: float
    upper_rad: float

    @property
    def origin_transform(self) -> np.ndarray:
        return transform(rotation_rpy(self.origin_rpy_rad), self.origin_xyz_m)


class WA100Kinematics:
    """Forward kinematics driven by six WA100 motor position units."""

    def __init__(self, model: Mapping[str, Any]):
        self.model = dict(model)
        if int(self.model.get("schema_version", 0)) != 1:
            raise ValueError("unsupported WA100 kinematics schema")
        self.root_link = str(self.model["root_link"])
        self.channel_names = tuple(self.model["calibration"]["channel_names"])
        self.channel_index = {name: index for index, name in enumerate(self.channel_names)}
        if len(self.channel_names) != 6 or len(self.channel_index) != 6:
            raise ValueError("WA100 model must define six unique channels")
        self.finger_chains = {name: tuple(chain) for name, chain in self.model["calibration"]["finger_chains"].items()}
        self.tip_frames = dict(self.model["calibration"]["tip_frames"])
        self.joint_mappings = dict(self.model["calibration"]["joint_mappings"])
        self.joints = {
            item["name"]: JointGeometry(
                name=item["name"], joint_type=item["joint_type"],
                parent_link=item["parent_link"], child_link=item["child_link"],
                origin_xyz_m=tuple(item["origin_xyz_m"]), origin_rpy_rad=tuple(item["origin_rpy_rad"]),
                axis=tuple(item["axis"]), lower_rad=float(item["lower_rad"]), upper_rad=float(item["upper_rad"]),
            )
            for item in self.model["joints"]
        }
        self._validate()

    @classmethod
    def load(cls, path: str | Path) -> "WA100Kinematics":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def _validate(self) -> None:
        mapped = set(self.joint_mappings)
        if mapped != set(self.joints):
            raise ValueError(f"joint mapping mismatch: missing={set(self.joints)-mapped}, extra={mapped-set(self.joints)}")
        deadzones = self.model["calibration"].get("channel_deadzones", {})
        if deadzones and set(deadzones) != set(self.channel_names):
            raise ValueError("endpoint deadzones must cover all six motor channels")
        for channel, settings in deadzones.items():
            open_deadzone = float(settings["open"])
            close_deadzone = float(settings["close"])
            if open_deadzone < 0.0 or close_deadzone < 0.0 or open_deadzone + close_deadzone >= 1.0:
                raise ValueError(f"invalid endpoint deadzones for channel: {channel}")
        for finger, chain in self.finger_chains.items():
            if finger not in self.tip_frames or any(name not in self.joints for name in chain):
                raise ValueError(f"invalid finger chain: {finger}")
            parent = self.root_link
            for name in chain:
                joint = self.joints[name]
                if joint.parent_link != parent:
                    raise ValueError(f"disconnected chain at {name}: expected parent {parent}")
                parent = joint.child_link
            if self.tip_frames[finger]["parent_link"] != parent:
                raise ValueError(f"tip parent mismatch for {finger}")
        thumb_model = self.model["calibration"].get("thumb_motion_model")
        if thumb_model:
            yaw_joint = str(thumb_model["yaw_joint"])
            pitch_joints = tuple(str(name) for name in thumb_model["pitch_joints"])
            if yaw_joint not in self.joints or len(pitch_joints) != 2:
                raise ValueError("thumb motion model requires one yaw joint and two pitch joints")
            if any(name not in self.joints for name in pitch_joints):
                raise ValueError("thumb motion model references an unknown pitch joint")
            pitch_channels = {self.joint_mappings[name]["channel"] for name in pitch_joints}
            if len(pitch_channels) != 1:
                raise ValueError("thumb J2/J3 must use the same coupled pitch channel")

            zero_angles = {name: 0.0 for name in self.joints}
            zero_links = self.link_transforms(zero_angles)

            def axis_in_root(name: str) -> np.ndarray:
                joint = self.joints[name]
                parent_rotation = zero_links[joint.parent_link][:3, :3]
                axis = parent_rotation @ rotation_rpy(joint.origin_rpy_rad) @ np.asarray(joint.axis, dtype=float)
                return axis / np.linalg.norm(axis)

            yaw_axis = axis_in_root(yaw_joint)
            pitch_axes = [axis_in_root(name) for name in pitch_joints]
            for name, pitch_axis in zip(pitch_joints, pitch_axes):
                axis_dot = float(np.dot(yaw_axis, pitch_axis))
                if abs(axis_dot) > 1e-5:
                    raise ValueError(f"thumb yaw and {name} flexion axes must be orthogonal; dot={axis_dot}")
            pitch_axis_dot = float(np.dot(pitch_axes[0], pitch_axes[1]))
            if abs(abs(pitch_axis_dot) - 1.0) > 1e-5:
                raise ValueError(f"thumb J2/J3 flexion axes must be parallel; dot={pitch_axis_dot}")
            pitch_signs = np.asarray(thumb_model["pitch_angle_signs"], dtype=float).reshape(-1)
            skeleton_scales = thumb_model["skeleton_angle_scales"]
            lengths = np.asarray(thumb_model["segment_lengths_m"], dtype=float).reshape(-1)
            yaw_axis = _vector(thumb_model["yaw_axis"])
            zero_direction = _vector(thumb_model["zero_yaw_direction"])
            pitch_axis_zero = _vector(thumb_model["pitch_axis_at_zero_yaw"])
            dorsum_normal = _vector(thumb_model["dorsum_plane_normal"])
            encoder_zero_direction = _vector(thumb_model["encoder_zero_direction"])
            if pitch_signs.size != 2 or lengths.size != 3 or np.any(lengths <= 0):
                raise ValueError("thumb model requires two pitch signs and three positive segment lengths")
            if any(float(skeleton_scales.get(name, 0.0)) <= 0 for name in (yaw_joint, *pitch_joints)):
                raise ValueError("thumb skeleton angle scales must be positive")
            yaw_axis /= np.linalg.norm(yaw_axis)
            zero_direction /= np.linalg.norm(zero_direction)
            pitch_axis_zero /= np.linalg.norm(pitch_axis_zero)
            dorsum_normal /= np.linalg.norm(dorsum_normal)
            encoder_zero_direction /= np.linalg.norm(encoder_zero_direction)
            if max(abs(np.dot(yaw_axis, zero_direction)), abs(np.dot(pitch_axis_zero, zero_direction)), abs(np.dot(yaw_axis, pitch_axis_zero))) > 1e-8:
                raise ValueError("thumb yaw, pitch and zero-link directions must be mutually orthogonal")
            if abs(abs(float(np.dot(dorsum_normal, encoder_zero_direction))) - 1.0) > 1e-8:
                raise ValueError("thumb encoder-zero direction must be perpendicular to the dorsum plane")

    def motor_units_to_joint_angles(
        self,
        motor_units: Sequence[float],
        *,
        clip_limits: bool = True,
        apply_deadzones: bool = True,
    ) -> dict[str, float]:
        units = np.asarray(motor_units, dtype=float).reshape(-1)
        if units.size != 6 or not np.all(np.isfinite(units)):
            raise ValueError("motor_units must contain six finite values")
        effective_units = units.copy()
        if apply_deadzones:
            deadzones = self.model["calibration"].get("channel_deadzones", {})
            for channel, settings in deadzones.items():
                if channel not in self.channel_index:
                    raise ValueError(f"deadzone references unknown channel: {channel}")
                open_deadzone = float(settings["open"])
                close_deadzone = float(settings["close"])
                if open_deadzone < 0.0 or close_deadzone < 0.0 or open_deadzone + close_deadzone >= 1.0:
                    raise ValueError(f"invalid endpoint deadzones for channel: {channel}")
                index = self.channel_index[channel]
                # Hardware convention: 2000 is open/zero-angle and 0 is fully
                # closed.  This is the same endpoint deadzone + smoothstep used
                # by udp_receiver_unity.cpp before it emits motor units.
                bent_ratio = float(np.clip((2000.0 - units[index]) / 2000.0, 0.0, 1.0))
                if bent_ratio <= open_deadzone:
                    motion_ratio = 0.0
                elif bent_ratio >= 1.0 - close_deadzone:
                    motion_ratio = 1.0
                else:
                    middle = (bent_ratio - open_deadzone) / (1.0 - open_deadzone - close_deadzone)
                    motion_ratio = middle * middle * (3.0 - 2.0 * middle)
                effective_units[index] = 2000.0 * (1.0 - motion_ratio)
        result: dict[str, float] = {}
        for name, mapping in self.joint_mappings.items():
            channel = str(mapping["channel"])
            samples = np.asarray(mapping["samples_units_rad"], dtype=float)
            if samples.ndim != 2 or samples.shape[1] != 2 or len(samples) < 2:
                raise ValueError(f"invalid calibration samples for {name}")
            samples = samples[np.argsort(samples[:, 0])]
            angle = float(np.interp(effective_units[self.channel_index[channel]], samples[:, 0], samples[:, 1]))
            joint = self.joints[name]
            result[name] = float(np.clip(angle, joint.lower_rad, joint.upper_rad)) if clip_limits else angle
        return result

    def link_transforms(self, joint_angles: Mapping[str, float], base_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        links = {self.root_link: np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float).reshape(4, 4)}
        unresolved = dict(self.joints)
        while unresolved:
            progressed = False
            for name, joint in list(unresolved.items()):
                if joint.parent_link not in links:
                    continue
                angle = float(joint_angles.get(name, 0.0))
                links[joint.child_link] = links[joint.parent_link] @ joint.origin_transform @ transform(rotation_axis(joint.axis, angle))
                del unresolved[name]
                progressed = True
            if not progressed:
                raise ValueError(f"URDF tree is disconnected: {sorted(unresolved)}")
        return links

    def skeleton_from_joint_angles(self, joint_angles: Mapping[str, float], base_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        links = self.link_transforms(joint_angles, base_transform)
        root_point = links[self.root_link][:3, 3]
        skeleton: dict[str, np.ndarray] = {}
        for finger, chain in self.finger_chains.items():
            points = [root_point]
            for name in chain:
                points.append(links[self.joints[name].child_link][:3, 3])
            tip = self.tip_frames[finger]
            tip_transform = links[tip["parent_link"]] @ transform(rotation_rpy(tip.get("rpy_rad", (0, 0, 0))), tip["xyz_m"])
            points.append(tip_transform[:3, 3])
            skeleton[finger] = np.asarray(points, dtype=float)
        thumb_model = self.model["calibration"].get("thumb_motion_model")
        if thumb_model:
            yaw_joint = str(thumb_model["yaw_joint"])
            pitch_joints = tuple(str(name) for name in thumb_model["pitch_joints"])
            pitch_signs = np.asarray(thumb_model["pitch_angle_signs"], dtype=float)
            skeleton_scales = thumb_model["skeleton_angle_scales"]
            lengths = np.asarray(thumb_model["segment_lengths_m"], dtype=float)
            base = _vector(thumb_model["base_xyz_m"])
            yaw_axis = _vector(thumb_model["yaw_axis"])
            zero_direction = _vector(thumb_model["zero_yaw_direction"])
            pitch_axis_zero = _vector(thumb_model["pitch_axis_at_zero_yaw"])
            yaw_angle = float(joint_angles.get(yaw_joint, 0.0)) * float(skeleton_scales[yaw_joint])
            yaw_rotation = rotation_axis(yaw_axis, yaw_angle)
            first_direction = yaw_rotation @ zero_direction
            pitch_axis = yaw_rotation @ pitch_axis_zero
            effective_pitch = [
                float(joint_angles.get(name, 0.0)) * float(skeleton_scales[name]) * float(sign)
                for name, sign in zip(pitch_joints, pitch_signs)
            ]
            second_direction = rotation_axis(pitch_axis, effective_pitch[0]) @ first_direction
            third_direction = rotation_axis(pitch_axis, sum(effective_pitch)) @ first_direction
            local_features = np.vstack([
                base,
                base + lengths[0] * first_direction,
                base + lengths[0] * first_direction + lengths[1] * second_direction,
                base + lengths[0] * first_direction + lengths[1] * second_direction + lengths[2] * third_direction,
            ])
            root_transform = links[self.root_link]
            world_features = (root_transform[:3, :3] @ local_features.T).T + root_transform[:3, 3]
            skeleton["thumb"] = np.vstack([root_point, world_features])
        return skeleton

    def skeleton_from_motor_units(self, motor_units: Sequence[float], base_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        return self.skeleton_from_joint_angles(self.motor_units_to_joint_angles(motor_units), base_transform)

    def thumb_feature_points_from_joint_angles(
        self,
        joint_angles: Mapping[str, float],
        base_transform: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return the four physical thumb points: base, bend joint, distal joint and tip."""
        return self.skeleton_from_joint_angles(joint_angles, base_transform)["thumb"][1:].copy()

    def thumb_feature_points_from_motor_units(
        self,
        motor_units: Sequence[float],
        base_transform: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return the four physical thumb points from the six WA100 motor units."""
        return self.thumb_feature_points_from_joint_angles(
            self.motor_units_to_joint_angles(motor_units), base_transform,
        )

    def fingertip_positions(self, motor_units: Sequence[float], base_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        return {finger: points[-1].copy() for finger, points in self.skeleton_from_motor_units(motor_units, base_transform).items()}


def parse_urdf_geometry(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    root = ET.parse(path).getroot()
    links = [element.get("name") for element in root.findall("link")]
    child_links = set()
    joints = []
    for element in root.findall("joint"):
        origin, axis, limit = element.find("origin"), element.find("axis"), element.find("limit")
        parent_link = element.find("parent").get("link")
        child_link = element.find("child").get("link")
        child_links.add(child_link)
        joints.append({
            "name": element.get("name"), "joint_type": element.get("type"),
            "parent_link": parent_link, "child_link": child_link,
            "origin_xyz_m": _vector(origin.get("xyz") if origin is not None else None).tolist(),
            "origin_rpy_rad": _vector(origin.get("rpy") if origin is not None else None).tolist(),
            "axis": _vector(axis.get("xyz") if axis is not None else None, (1, 0, 0)).tolist(),
            "lower_rad": float(limit.get("lower", 0.0)) if limit is not None else 0.0,
            "upper_rad": float(limit.get("upper", 0.0)) if limit is not None else 0.0,
        })
    roots = sorted(set(links) - child_links)
    if len(roots) != 1:
        raise ValueError(f"expected one URDF root link, found {roots}")
    return {
        "robot_name": root.get("name"), "root_link": roots[0], "links": links,
        "joints": joints,
        "source_urdf": str(path.resolve()),
        "source_urdf_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def compile_model(urdf_path: str | Path, calibration_path: str | Path, output_path: str | Path) -> dict[str, Any]:
    geometry = parse_urdf_geometry(urdf_path)
    calibration_path = Path(calibration_path)
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    model = {
        "schema_version": 1,
        **geometry,
        "calibration": calibration,
        "source_calibration": str(calibration_path.resolve()),
        "source_calibration_sha256": hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
    }
    WA100Kinematics(model)  # validate before writing
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(model, ensure_ascii=False, indent=2), encoding="utf-8")
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description="Compile or inspect the local WA100 kinematics model")
    subparsers = parser.add_subparsers(dest="command", required=True)
    compile_parser = subparsers.add_parser("compile")
    compile_parser.add_argument("--urdf", type=Path, required=True)
    compile_parser.add_argument("--calibration", type=Path, required=True)
    compile_parser.add_argument("--output", type=Path, required=True)
    inspect_parser = subparsers.add_parser("inspect")
    inspect_parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "compile":
        model = compile_model(args.urdf, args.calibration, args.output)
        print(json.dumps({"output": str(args.output.resolve()), "links": len(model["links"]), "joints": len(model["joints"]), "sha256": model["source_urdf_sha256"]}, indent=2))
    else:
        kinematics = WA100Kinematics.load(args.model)
        print(json.dumps({
            "robot_name": kinematics.model["robot_name"], "root_link": kinematics.root_link,
            "links": len(kinematics.model["links"]), "joints": len(kinematics.joints),
            "channels": kinematics.channel_names,
            "calibration_id": kinematics.model["calibration"].get("calibration_id"),
            "calibration_status": kinematics.model["calibration"].get("status"),
        }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
