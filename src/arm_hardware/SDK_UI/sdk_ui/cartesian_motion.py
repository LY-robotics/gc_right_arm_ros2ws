"""Execute and record sampled Cartesian moves through the existing position SDK."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import math
import os
from pathlib import Path
import threading
import time
from typing import Callable, Sequence, TYPE_CHECKING, TextIO

if TYPE_CHECKING:
    from .arm_sdk import ArmSdk, PositionSnapshot
    from .kinematics import ArmKinematics, CartesianPose


_SAMPLE_PERIOD_S = 0.05
_START_SETTLE_S = 0.10
_START_POSITION_TOLERANCE_M = 0.0003
_START_JOINT_TOLERANCE_DEG = 0.05
_POSITION_TOLERANCE_M = 0.001
_PATH_TOLERANCE_M = 0.003
_BACKTRACK_TOLERANCE_M = 0.001
_JOINT_TOLERANCE_DEG = 0.12
_ARRIVAL_ORIENTATION_RAD = math.radians(1.0)
_PATH_ORIENTATION_RAD = math.radians(3.0)
_SETTLE_SAMPLES = 2

_TRACE_HEADER = (
    "event", "detail", "timestamp_utc", "elapsed_s", "frame_id", "waypoint_index",
    "target_x_m", "target_y_m", "target_z_m", "x_m", "y_m", "z_m",
    "qx", "qy", "qz", "qw", "roll_deg", "pitch_deg", "yaw_deg",
    "vx_m_s", "vy_m_s", "vz_m_s", "linear_speed_m_s",
    "wx_rad_s", "wy_rad_s", "wz_rad_s", "angular_speed_rad_s", "cross_track_error_m",
    "joint1_deg", "joint2_deg", "joint3_deg", "joint4_deg", "joint5_deg", "joint6_deg",
)


@dataclass(frozen=True)
class CartesianSample:
    elapsed_s: float
    waypoint_index: int
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    rpy_deg: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float] | None
    speed_m_s: float | None
    angular_velocity_rad_s: tuple[float, float, float] | None
    angular_speed_rad_s: float | None
    cross_track_error_m: float
    joints_deg: tuple[float, ...]


@dataclass(frozen=True)
class CartesianMotionResult:
    trace_path: Path
    samples: int
    waypoints_completed: int
    status: str


class CartesianMotionError(RuntimeError):
    def __init__(self, detail: str, trace_path: Path | None = None) -> None:
        self.trace_path = trace_path
        super().__init__(detail if trace_path is None else f"{detail}。轨迹文件 {trace_path}")


def _angular_distance(a: Sequence[float], b: Sequence[float]) -> float:
    dot = sum(x * y for x, y in zip(a, b))
    return 2.0 * math.acos(min(1.0, abs(dot)))


def _angular_velocity(
    previous: Sequence[float], current: Sequence[float], dt: float
) -> tuple[float, float, float]:
    # current * conjugate(previous) gives a relative rotation in the base frame.
    px, py, pz, pw = previous
    x, y, z, w = current
    vector = (
        pw * x - w * px + py * z - pz * y,
        pw * y - w * py + pz * x - px * z,
        pw * z - w * pz + px * y - py * x,
    )
    norm = math.hypot(*vector)
    if norm == 0:
        return 0.0, 0.0, 0.0
    scalar = sum(a * b for a, b in zip(previous, current))
    scale = math.copysign(2.0 * math.atan2(norm, abs(scalar)) / (norm * dt), scalar)
    return scale * vector[0], scale * vector[1], scale * vector[2]


def _distance_to_line_segment(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    direction = tuple(b - a for a, b in zip(start, end))
    length_squared = sum(value * value for value in direction)
    if length_squared == 0:
        return math.dist(start, point)
    fraction = sum((value - a) * step for value, a, step in zip(point, start, direction))
    fraction = min(1.0, max(0.0, fraction / length_squared))
    return math.dist(point, (a + fraction * step for a, step in zip(start, direction)))


def _check_cancelled(cancel_event: threading.Event) -> None:
    if cancel_event.is_set():
        raise InterruptedError("末端直线运动已取消")


def _check_stationary_start(
    start_snapshot: PositionSnapshot,
    start_pose: CartesianPose,
    snapshot: PositionSnapshot,
    pose: CartesianPose,
) -> None:
    if (math.dist(start_pose.position_m, pose.position_m) > _START_POSITION_TOLERANCE_M
            or max(abs(a - b) for a, b in zip(start_snapshot.joint_deg, snapshot.joint_deg))
            > _START_JOINT_TOLERANCE_DEG):
        raise RuntimeError("起点仍在移动，请等待机械臂静止后重新规划")


class _Trace:
    def __init__(self, stream: TextIO, target_m: tuple[float, ...]) -> None:
        self._stream = stream
        self._writer = csv.writer(stream)
        self._writer.writerow(_TRACE_HEADER)
        self._target_m = target_m
        self.start_time = time.perf_counter()
        self._start_utc = datetime.now(timezone.utc)
        self.previous: CartesianSample | None = None
        self.samples = 0

    def _prefix(self, event: str, detail: str, elapsed: float, waypoint: int) -> tuple:
        stamp = self._start_utc + timedelta(seconds=elapsed)
        return event, detail, stamp.isoformat(), elapsed, "base_link", waypoint, *self._target_m

    def write_sample(
        self, elapsed: float, waypoint: int, pose: CartesianPose, joints_deg: tuple[float, ...],
        cross_track: float,
    ) -> CartesianSample:
        linear = angular = None
        if self.previous is not None:
            dt = elapsed - self.previous.elapsed_s
            if dt > 0:
                px, py, pz = self.previous.position_m
                x, y, z = pose.position_m
                linear = (x - px) / dt, (y - py) / dt, (z - pz) / dt
                angular = _angular_velocity(self.previous.quaternion_xyzw, pose.quaternion_xyzw, dt)
        sample = CartesianSample(
            elapsed, waypoint, pose.position_m, pose.quaternion_xyzw, pose.rpy_deg,
            linear, math.hypot(*linear) if linear is not None else None,
            angular, math.hypot(*angular) if angular is not None else None, cross_track, joints_deg,
        )
        self._writer.writerow((
            *self._prefix("sample", "", elapsed, waypoint), *pose.position_m, *pose.quaternion_xyzw,
            *pose.rpy_deg, *(linear or (None,) * 3), sample.speed_m_s,
            *(angular or (None,) * 3), sample.angular_speed_rad_s, cross_track, *joints_deg,
        ))
        self._stream.flush()
        self.previous = sample
        self.samples += 1
        return sample

    def finish(self, status: str, detail: str, completed: int) -> None:
        prefix = self._prefix(status, detail, time.perf_counter() - self.start_time, completed)
        self._writer.writerow((*prefix, *([None] * (len(_TRACE_HEADER) - len(prefix)))))
        self._stream.flush()
        os.fsync(self._stream.fileno())


def execute_cartesian_line(
    sdk: ArmSdk,
    model: ArmKinematics,
    target_xyz_m: Sequence[float],
    cartesian_speed_m_s: float,
    joint_speed_limit_dps: float,
    trace_path: str | Path,
    *,
    cancel_event: threading.Event,
    on_sample: Callable[[CartesianSample], None] | None = None,
) -> CartesianMotionResult:
    """Run in a worker thread. Callbacks receive samples on that same thread."""
    try:
        target = tuple(float(value) for value in target_xyz_m)
    except (ValueError, TypeError, OverflowError) as error:
        raise CartesianMotionError("目标位置必须是三个有限的 XYZ 坐标") from error
    if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise CartesianMotionError("目标位置必须是三个有限的 XYZ 坐标")
    if not math.isfinite(cartesian_speed_m_s) or cartesian_speed_m_s <= 0:
        raise CartesianMotionError("末端速度必须是有限正数")
    if not math.isfinite(joint_speed_limit_dps) or joint_speed_limit_dps <= 0:
        raise CartesianMotionError("关节速度上限必须是有限正数")
    joint_speed_limit_dps = min(joint_speed_limit_dps, *model.velocity_limits_dps)

    path = Path(trace_path).expanduser().absolute()
    created_path: Path | None = None
    started = False
    completed = 0
    failure: Exception | None = None
    status = "completed"
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8", newline="") as stream:
            created_path = path
            trace = _Trace(stream, target)
            furthest_m = 0.0

            def read() -> tuple[PositionSnapshot, CartesianPose, float]:
                before = time.perf_counter()
                snapshot = sdk.read_positions(cancel_event=cancel_event)
                elapsed = (before + time.perf_counter()) / 2 - trace.start_time
                return snapshot, model.forward(snapshot.joint_deg), elapsed

            def record(
                index: int, snapshot: PositionSnapshot, pose: CartesianPose, elapsed: float
            ) -> None:
                nonlocal furthest_m
                cross_track = _distance_to_line_segment(pose.position_m, start_pose.position_m, target)
                sample = trace.write_sample(elapsed, index, pose, snapshot.joint_deg, cross_track)
                if on_sample is not None:
                    on_sample(sample)
                if cross_track > _PATH_TOLERANCE_M:
                    raise RuntimeError(f"末端偏离目标线段 {cross_track * 1000:.2f} mm")
                if _angular_distance(pose.quaternion_xyzw, start_pose.quaternion_xyzw) > _PATH_ORIENTATION_RAD:
                    raise RuntimeError("末端姿态偏离起始姿态超过 3°")
                progress = sum((p - s) * direction for p, s, direction in zip(
                    pose.position_m, start_pose.position_m, line_direction
                ))
                if progress < furthest_m - _BACKTRACK_TOLERANCE_M:
                    raise RuntimeError("末端沿直线方向回退超过 1 mm")
                furthest_m = max(furthest_m, progress)

            try:
                _check_cancelled(cancel_event)
                start_snapshot, start_pose, elapsed = read()
                line_length = math.dist(start_pose.position_m, target)
                line_direction = tuple((b - a) / line_length if line_length else 0.0
                                       for a, b in zip(start_pose.position_m, target))
                record(0, start_snapshot, start_pose, elapsed)
                cancel_event.wait(_START_SETTLE_S)
                _check_cancelled(cancel_event)
                current, pose, elapsed = read()
                record(0, current, pose, elapsed)
                _check_stationary_start(start_snapshot, start_pose, current, pose)
                try:
                    waypoints = model.plan_line(
                        start_snapshot.joint_deg, target,
                        joint_limits_deg=sdk.config["joint_position_limits_deg"],
                        cancelled=cancel_event.is_set,
                    )
                except Exception:
                    _check_cancelled(cancel_event)
                    raise
                for waypoint in waypoints:
                    _check_cancelled(cancel_event)
                    sdk.check_absolute_target(waypoint.joint_deg)

                if waypoints:
                    current, pose, elapsed = read()
                    record(0, current, pose, elapsed)
                    _check_stationary_start(start_snapshot, start_pose, current, pose)

                segment_start = start_pose.position_m
                for index, waypoint in enumerate(waypoints, start=1):
                    _check_cancelled(cancel_event)
                    duration = max(waypoint.distance_m, math.dist(pose.position_m, waypoint.position_m))
                    duration /= cartesian_speed_m_s
                    joint_distance = max(abs(a - b) for a, b in zip(current.joint_deg, waypoint.joint_deg))
                    speed = joint_speed_limit_dps
                    if joint_distance > 0 and duration > 0:
                        speed = min(speed, joint_distance / duration)
                    started = True
                    move = sdk.move_absolute_from_snapshot(
                        current, waypoint.joint_deg, speed, cancel_event=cancel_event
                    )
                    deadline = time.perf_counter() + max(3.0, 3.0 * move.duration_s + 1.0)
                    settled = 0
                    while settled < _SETTLE_SAMPLES:
                        _check_cancelled(cancel_event)
                        read_started = time.perf_counter()
                        current, pose, elapsed = read()
                        record(index, current, pose, elapsed)
                        segment_error = _distance_to_line_segment(
                            pose.position_m, segment_start, waypoint.position_m
                        )
                        if segment_error > _PATH_TOLERANCE_M:
                            raise RuntimeError(f"末端超出第 {index} 个直线子段范围")
                        if time.perf_counter() >= deadline:
                            raise TimeoutError(f"末端直线运动第 {index} 个路径点到位超时")
                        arrived = (
                            math.dist(pose.position_m, waypoint.position_m) <= _POSITION_TOLERANCE_M
                            and _angular_distance(pose.quaternion_xyzw, waypoint.quaternion_xyzw)
                            <= _ARRIVAL_ORIENTATION_RAD
                            and max(abs(a - b) for a, b in zip(current.joint_deg, waypoint.joint_deg))
                            <= _JOINT_TOLERANCE_DEG
                        )
                        settled = settled + 1 if arrived else 0
                        if settled < _SETTLE_SAMPLES:
                            cancel_event.wait(max(0.0, _SAMPLE_PERIOD_S - (time.perf_counter() - read_started)))
                    completed = index
                    segment_start = waypoint.position_m
                _check_cancelled(cancel_event)
            except Exception as error:
                failure = error
                status = "cancelled" if isinstance(error, InterruptedError) and cancel_event.is_set() else "failed"
                if started:
                    try:
                        sdk.emergency_stop()
                    except Exception as stop_error:
                        failure = RuntimeError(f"{error}；急停未全部确认：{stop_error}")
                        status = "failed"
            trace.finish(status, str(failure) if failure is not None else "", completed)
    except Exception as error:
        detail = f"{failure}；轨迹写入失败：{error}" if failure is not None else str(error)
        raise CartesianMotionError(detail, created_path) from error

    if status == "failed":
        raise CartesianMotionError(str(failure), path) from failure
    return CartesianMotionResult(path, trace.samples, completed, status)
