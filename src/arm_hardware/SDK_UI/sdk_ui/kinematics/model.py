"""Kinematics of Joint1…Joint6 from the mechanical arm's URDF."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, ceil, cos, degrees, isfinite, radians, sin, sqrt
import os
from pathlib import Path
from typing import Callable, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray

from ..tendon_compensation import package_file, validate_j2_urdf_direction

_Array = NDArray[np.float64]
_POSITION_TOL_M = 0.00005
_ANGLE_TOL_RAD = radians(0.02)
_ORIENTATION_WEIGHT_M = 0.20
_MAX_WAYPOINTS = 1000


class KinematicsError(ValueError):
    """A Cartesian operation could not be planned."""


@dataclass(frozen=True)
class CartesianPose:
    position_m: tuple[float, ...]
    quaternion_xyzw: tuple[float, ...]
    rpy_deg: tuple[float, ...]


@dataclass(frozen=True)
class LineWaypoint:
    joint_deg: tuple[float, ...]
    position_m: tuple[float, ...]
    distance_m: float
    quaternion_xyzw: tuple[float, ...]


@dataclass(frozen=True)
class _Joint:
    translation_m: _Array
    origin_rotation: _Array
    axis: _Array
    lower_rad: float
    upper_rad: float
    velocity_rad_s: float


def _finite_array(values, shape: tuple[int, ...], label: str) -> _Array:
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise KinematicsError(f"{label}包含无效数值") from error
    if array.shape != shape or not np.isfinite(array).all():
        raise KinematicsError(f"{label}需要形状为 {shape} 的有限数值")
    return array


def _as_tuple(array: _Array) -> tuple[float, ...]:
    return tuple(float(value) for value in array)


def _rotation_about(unit_axis: _Array, angle: float) -> _Array:
    x, y, z = unit_axis
    skew = np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))
    cosine = cos(angle)
    return cosine * np.eye(3) + (1 - cosine) * np.outer(unit_axis, unit_axis) + sin(angle) * skew


def _origin_rotation(rpy: _Array) -> _Array:
    axes = np.eye(3)
    return _rotation_about(axes[2], rpy[2]) @ _rotation_about(
        axes[1], rpy[1]
    ) @ _rotation_about(axes[0], rpy[0])


def _quaternion(rotation: _Array) -> _Array:
    # Recover the largest quaternion component first, including near 180°.
    diagonal = rotation.diagonal()
    trace = float(diagonal.sum())
    components_squared = np.append(1 + 2 * diagonal - trace, 1 + trace) / 4
    index = int(np.argmax(components_squared))
    component = sqrt(components_squared[index])
    q = np.zeros(4)
    q[index] = component
    if index == 3:
        q[:3] = np.array((rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                          rotation[1, 0] - rotation[0, 1])) / (4 * component)
    else:
        j, k = (index + 1) % 3, (index + 2) % 3
        q[j] = (rotation[j, index] + rotation[index, j]) / (4 * component)
        q[k] = (rotation[k, index] + rotation[index, k]) / (4 * component)
        q[3] = (rotation[k, j] - rotation[j, k]) / (4 * component)
    if q[3] < 0:
        q = -q
    return q / np.linalg.norm(q)


def _rotation_vector(rotation: _Array) -> _Array:
    quaternion = _quaternion(rotation)
    sine = np.linalg.norm(quaternion[:3])
    if sine < 1e-12:
        return 2 * quaternion[:3]
    return quaternion[:3] * (2 * atan2(sine, quaternion[3]) / sine)


def _quaternion_rotation(values: Sequence[float]) -> _Array:
    q = _finite_array(values, (4,), "末端四元数")
    scale = np.max(np.abs(q))
    if scale == 0:
        raise KinematicsError("末端四元数的模必须大于零")
    q = q / scale
    q = q / np.linalg.norm(q)
    sine = np.linalg.norm(q[:3])
    if sine == 0:
        return np.eye(3)
    return _rotation_about(q[:3] / sine, 2 * atan2(sine, q[3]))


def _rpy_deg(rotation: _Array) -> tuple[float, ...]:
    cosine_pitch = sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    pitch = atan2(-rotation[2, 0], cosine_pitch)
    if cosine_pitch > 1e-9:
        roll = atan2(rotation[2, 1], rotation[2, 2])
        yaw = atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0
        yaw = atan2(-rotation[0, 1], rotation[1, 1])
    return tuple(map(degrees, (roll, pitch, yaw)))


def _read_joints(urdf: Path) -> tuple[str, tuple[_Joint, ...]]:
    try:
        robot = ET.parse(urdf).getroot()
        links = {link.get("name") for link in robot.findall("./link")}
        base = "base_link" if "base_link" in links else "base"
        previous_link = base
        joints = []
        for index in range(1, 7):
            joint = robot.find(f"./joint[@name='Joint{index}']")
            if joint is None or joint.get("type") != "revolute":
                raise KinematicsError(f"URDF 中缺少旋转关节 Joint{index}")
            parent, child = joint.find("parent"), joint.find("child")
            link = f"Arm{index}"
            if (previous_link not in links or link not in links or parent is None or child is None
                    or parent.get("link") != previous_link or child.get("link") != link):
                raise KinematicsError(f"URDF 中 Joint{index} 不是预期的 {previous_link} → {link}")
            origin = joint.find("origin")
            axis_node = joint.find("axis")
            xyz = "0 0 0" if origin is None else origin.get("xyz", "0 0 0")
            rpy = "0 0 0" if origin is None else origin.get("rpy", "0 0 0")
            axis_xyz = "1 0 0" if axis_node is None else axis_node.get("xyz", "1 0 0")
            axis = _finite_array(axis_xyz.split(), (3,), f"Joint{index} 转轴")
            axis_norm = np.linalg.norm(axis)
            limit = joint.find("limit")
            if not isfinite(axis_norm) or axis_norm == 0 or limit is None:
                raise KinematicsError(f"URDF 中 Joint{index} 的转轴或限位无效")
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            if not (isfinite(lower) and isfinite(upper) and lower < upper):
                raise KinematicsError(f"URDF 中 Joint{index} 的关节限位无效")
            velocity = float(limit.attrib["velocity"])
            if not isfinite(velocity) or velocity <= 0:
                raise KinematicsError(f"URDF 中 Joint{index} 的速度限位无效")
            joints.append(_Joint(
                _finite_array(xyz.split(), (3,), f"Joint{index} 原点"),
                _origin_rotation(_finite_array(rpy.split(), (3,), f"Joint{index} RPY")),
                axis / axis_norm, lower, upper, velocity,
            ))
            previous_link = link
    except KinematicsError:
        raise
    except (OSError, ET.ParseError, KeyError, ValueError) as error:
        raise KinematicsError(f"无法读取机械臂 URDF {urdf}：{error}") from error
    return base, tuple(joints)


class ArmKinematics:
    """FK and bounded numerical IK for SDK joint angles in the URDF base frame."""

    def __init__(
        self, urdf_path: str | Path | None = None, *, j2_urdf_direction: int = 1,
    ) -> None:
        try:
            j2_urdf_direction = validate_j2_urdf_direction(j2_urdf_direction)
        except ValueError as error:
            raise KinematicsError(str(error)) from error
        override = urdf_path or os.environ.get("ARM_TENDON_URDF")
        self.urdf_path = Path(override).expanduser().resolve() if override else package_file(
            "arm_description", "urdf/Arm_R.urdf.xacro"
        )
        self.base_frame, self._joints = _read_joints(self.urdf_path)
        if j2_urdf_direction == -1:
            # Express J2's axis and bounds in SDK coordinates for both FK and IK.
            joint2 = self._joints[1]
            joint2 = replace(
                joint2, axis=-joint2.axis, lower_rad=-joint2.upper_rad, upper_rad=-joint2.lower_rad,
            )
            self._joints = (self._joints[0], joint2, *self._joints[2:])
        self.tip_frame = "Arm6"
        self._bounds = np.array([(joint.lower_rad, joint.upper_rad) for joint in self._joints])
        self._reach_m = sum(np.linalg.norm(joint.translation_m) for joint in self._joints[1:])
        self.position_limits_deg = tuple(_as_tuple(row) for row in np.rad2deg(self._bounds))
        self.velocity_limits_dps = tuple(degrees(joint.velocity_rad_s) for joint in self._joints)

    def _forward_rad(self, joints_rad: _Array) -> tuple[_Array, _Array, _Array]:
        position = np.zeros(3)
        rotation = np.eye(3)
        axes = np.empty((6, 3))
        origins = np.empty((6, 3))
        for index, (joint, angle) in enumerate(zip(self._joints, joints_rad)):
            position = position + rotation @ joint.translation_m
            rotation = rotation @ joint.origin_rotation
            origins[index] = position
            axes[index] = rotation @ joint.axis
            rotation = rotation @ _rotation_about(joint.axis, angle)
        jacobian = np.vstack((np.cross(axes, position - origins).T, axes.T))
        return position, rotation, jacobian

    def forward(self, joint_deg: Sequence[float]) -> CartesianPose:
        """Return Arm6 position in metres and orientation in the base frame."""
        angles = np.deg2rad(_finite_array(joint_deg, (6,), "关节角"))
        position, rotation, _ = self._forward_rad(angles)
        return CartesianPose(_as_tuple(position), _as_tuple(_quaternion(rotation)), _rpy_deg(rotation))

    def _joint_bounds(self, joint_limits_deg: Sequence[Sequence[float]] | None) -> _Array:
        bounds = self._bounds.copy()
        if joint_limits_deg is not None:
            limits = np.deg2rad(_finite_array(joint_limits_deg, (6, 2), "SDK 关节限位"))
            bounds[:, 0] = np.maximum(bounds[:, 0], limits[:, 0])
            bounds[:, 1] = np.minimum(bounds[:, 1], limits[:, 1])
        if np.any(bounds[:, 0] >= bounds[:, 1]):
            raise KinematicsError("SDK 与 URDF 的关节限位没有有效交集")
        return bounds

    def _check_reach(self, position: _Array) -> None:
        if np.linalg.norm(position - self._joints[0].translation_m) > self._reach_m + _POSITION_TOL_M:
            raise KinematicsError("目标坐标超出机械臂最大伸展范围")

    @staticmethod
    def _check_cancelled(cancelled: Callable[[], bool] | None) -> None:
        if cancelled is not None and cancelled():
            raise KinematicsError("直线规划已取消")

    @staticmethod
    def _weighted_error(
        target_position: _Array, target_rotation: _Array, position: _Array, rotation: _Array
    ) -> _Array:
        return np.concatenate((
            target_position - position, _ORIENTATION_WEIGHT_M * _rotation_vector(target_rotation @ rotation.T),
        ))

    def _inverse_rad(
        self, target_position: _Array, target_rotation: _Array, seed_rad: _Array, bounds: _Array,
        cancelled: Callable[[], bool] | None = None,
    ) -> _Array:
        lower, upper = bounds.T
        joints = np.clip(seed_rad, lower, upper)
        for _ in range(70):
            self._check_cancelled(cancelled)
            position, rotation, jacobian = self._forward_rad(joints)
            error = self._weighted_error(target_position, target_rotation, position, rotation)
            if (np.linalg.norm(error[:3]) <= _POSITION_TOL_M
                    and np.linalg.norm(error[3:]) <= _ANGLE_TOL_RAD * _ORIENTATION_WEIGHT_M):
                return joints
            jacobian[3:] *= _ORIENTATION_WEIGHT_M
            free = np.ones(6, dtype=bool)
            step = np.zeros(6)
            # Re-solve when a joint already at its bound wants to move outwards.
            while free.any():
                u, singular, vt = np.linalg.svd(jacobian[:, free], full_matrices=False)
                damping = max(1e-5, 0.02 * (0.01 - singular[-1]))
                step.fill(0)
                step[free] = vt.T @ ((singular / (singular ** 2 + damping ** 2)) * (u.T @ error))
                blocked = free & (
                    ((joints <= lower + 1e-10) & (step < 0))
                    | ((joints >= upper - 1e-10) & (step > 0))
                )
                if not blocked.any():
                    break
                free[blocked] = False
            max_step = np.max(np.abs(step))
            if not free.any() or max_step < 1e-10:
                break
            step *= min(1.0, radians(8) / max_step)
            squared_error = error @ error
            for scale in (1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625):
                candidate = np.clip(joints + scale * step, lower, upper)
                candidate_position, candidate_rotation, _ = self._forward_rad(candidate)
                candidate_error = self._weighted_error(
                    target_position, target_rotation, candidate_position, candidate_rotation
                )
                if candidate_error @ candidate_error < squared_error:
                    joints = candidate
                    break
            else:
                break
        raise KinematicsError("当前关节分支无法在限位内达到该位置与姿态")

    def inverse(
        self, position_m: Sequence[float], quaternion_xyzw: Sequence[float],
        seed_joint_deg: Sequence[float], joint_limits_deg: Sequence[Sequence[float]] | None = None,
    ) -> tuple[float, ...]:
        """Solve for a pose near the given seed under the URDF/SDK limits."""
        position = _finite_array(position_m, (3,), "目标 XYZ")
        self._check_reach(position)
        rotation = _quaternion_rotation(quaternion_xyzw)
        seed = np.deg2rad(_finite_array(seed_joint_deg, (6,), "逆解种子"))
        result = self._inverse_rad(position, rotation, seed, self._joint_bounds(joint_limits_deg))
        return _as_tuple(np.rad2deg(result))

    def plan_line(
        self, start_joint_deg: Sequence[float], target_position_m: Sequence[float], *,
        joint_limits_deg: Sequence[Sequence[float]] | None = None, max_step_m: float = 0.005,
        cancelled: Callable[[], bool] | None = None,
    ) -> tuple[LineWaypoint, ...]:
        """Plan a sampled XYZ line from the measured pose with its orientation held."""
        self._check_cancelled(cancelled)
        target = _finite_array(target_position_m, (3,), "目标 XYZ")
        self._check_reach(target)
        if not isfinite(max_step_m) or max_step_m <= 0:
            raise KinematicsError("直线路点间距必须为有限正数")
        start = np.deg2rad(_finite_array(start_joint_deg, (6,), "反馈关节角"))
        bounds = self._joint_bounds(joint_limits_deg)
        outside = np.flatnonzero((start < bounds[:, 0] - 1e-10) | (start > bounds[:, 1] + 1e-10))
        if outside.size:
            details = "；".join(
                f"J{index + 1} 反馈 {degrees(start[index]):.9f}°，允许范围 "
                f"[{degrees(bounds[index, 0]):.9f}°, {degrees(bounds[index, 1]):.9f}°]"
                for index in outside
            )
            raise KinematicsError(f"反馈关节角超出关节限位，无法开始直线规划（{details}）")
        start_xyz, orientation, _ = self._forward_rad(start)
        displacement = target - start_xyz
        length = float(np.linalg.norm(displacement))
        if length <= _POSITION_TOL_M:
            return ()
        segments = length / max_step_m
        if segments > _MAX_WAYPOINTS:
            raise KinematicsError("直线路点过多，无法规划")
        segments = ceil(segments)
        quaternion = _as_tuple(_quaternion(orientation))
        waypoints: list[LineWaypoint] = []

        def extend(left: float, left_joints: _Array, right: float, depth: int) -> _Array:
            self._check_cancelled(cancelled)
            if len(waypoints) >= _MAX_WAYPOINTS:
                raise KinematicsError("直线路点过多，无法规划")
            point = start_xyz + right * displacement
            middle = (left + right) / 2
            try:
                joints = self._inverse_rad(point, orientation, left_joints, bounds, cancelled)
            except KinematicsError as error:
                self._check_cancelled(cancelled)
                detail = str(error)
            else:
                midpoint, midpoint_rotation, _ = self._forward_rad((left_joints + joints) / 2)
                if (np.max(np.abs(joints - left_joints)) <= radians(5)
                        and np.linalg.norm(midpoint - (start_xyz + middle * displacement)) <= 0.00025
                        and np.linalg.norm(_rotation_vector(orientation @ midpoint_rotation.T)) <= radians(0.25)):
                    waypoints.append(LineWaypoint(
                        _as_tuple(np.rad2deg(joints)), _as_tuple(point), (right - left) * length, quaternion
                    ))
                    return joints
                detail = "相邻路点的关节变化或中点偏差过大"
            if depth >= 6:
                raise KinematicsError(f"直线 {right:.1%} 处无法连续规划：{detail}")
            middle_joints = extend(left, left_joints, middle, depth + 1)
            return extend(middle, middle_joints, right, depth + 1)

        joints = start
        for index in range(segments):
            joints = extend(index / segments, joints, (index + 1) / segments, 0)
        return tuple(waypoints)
