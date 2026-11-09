"""Logical-joint SDK for the six coupled real motors."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
from typing import Any, Sequence
from uuid import uuid4

from .motor_coordinates import check_position_limits, joint_to_motor, motor_to_joint, quantize_a4
from .protocol import CanMotor, PidGains, SocketCanBus, Status1, Status2
from .tendon_compensation import (
    ENCODER_LIMIT_TOLERANCE_DEG, TendonEvaluation, TendonModel, validate_j2_urdf_direction,
)


PI_HISTORY_PATH = Path.home() / ".config" / "muweidu_arm_sdk_ui" / "pi_history.jsonl"


def _append_pi_history(event: dict[str, Any]) -> None:
    """Append one durable JSON record before/after a potentially persistent write."""
    import fcntl

    PI_HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
    record = (json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n").encode("utf-8")
    fd = os.open(PI_HISTORY_PATH, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        original_size = os.fstat(fd).st_size
        try:
            remaining = memoryview(record)
            while remaining:
                count = os.write(fd, remaining)
                if count <= 0:
                    raise OSError("could not append PI history record")
                remaining = remaining[count:]
            os.fsync(fd)
            # Ensure a newly created file's directory entry is durable as well.
            directory_fd = os.open(PI_HISTORY_PATH.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except Exception:
            # A failed partial append must not normally corrupt earlier JSONL rows.
            try:
                os.ftruncate(fd, original_size)
                os.fsync(fd)
            except OSError:
                pass
            raise
    finally:
        os.close(fd)


@dataclass(frozen=True)
class PositionSnapshot:
    raw_motor_deg: tuple[float, ...]
    mapped_motor_deg: tuple[float, ...]
    joint_deg: tuple[float, ...]
    encoder_equivalent_joint_deg: tuple[float, ...] | None = None


@dataclass(frozen=True)
class MoveResult:
    current_joint_deg: tuple[float, ...]
    target_joint_deg: tuple[float, ...]
    target_raw_motor_deg: tuple[float, ...]
    requested_motor_speed_dps: tuple[float, ...]
    applied_motor_speed_dps: tuple[int, ...]
    duration_s: float
    tendon: TendonEvaluation | None = None
    motor_compensation_deg: tuple[float, ...] | None = None
    predicted_joint_deg: tuple[float, ...] | None = None


@dataclass(frozen=True)
class _PreparedTarget:
    joint_deg: tuple[float, ...]
    raw_motor_deg: tuple[float, ...]
    speed_joint_deg: tuple[float, ...]
    speed_motor_deg: tuple[float, ...]
    tendon: TendonEvaluation | None
    motor_compensation_deg: tuple[float, ...] | None
    predicted_joint_deg: tuple[float, ...] | None


@dataclass(frozen=True)
class MotorZeroResult:
    """Acknowledged zero-offset write, or an unconfirmed write with its error."""

    motor_id: int
    encoder_zero_offset: int | None
    error: str | None


@dataclass(frozen=True)
class PidWriteResult:
    motor_id: int
    before: PidGains
    requested: PidGains
    observed: PidGains | None
    status: str
    error: str | None


class ArmSdk:
    """SDK methods operate on real hardware and accept logical joint degrees."""

    _tendon_model: TendonModel | None = None
    _last_tendon_estimate: tuple[float, ...] | None = None

    def __init__(self, hardware_config: dict[str, Any], timeout_s: float = 0.2) -> None:
        self.config = self._validate_config(hardware_config)
        self._tendon_model = self._load_tendon_model(self.config)
        self.timeout_s = timeout_s
        self.bus = SocketCanBus(self.config["can_interface"], self.timeout_s)
        self.motors = [CanMotor(self.bus, motor_id) for motor_id in self.config["motor_ids"]]
        self._operation_lock = threading.RLock()

    @staticmethod
    def _validate_config(config: dict[str, Any]) -> dict[str, Any]:
        result = dict(config)
        ids = [int(value) for value in result["motor_ids"]]
        if len(ids) != 6 or len(set(ids)) != 6 or any(value < 1 or value > 32 for value in ids):
            raise ValueError("motor_ids must contain six unique CAN motor IDs in range 1..32")
        for key in ("motor_directions", "motor_zero_offset_deg", "motor_speed_limit_dps"):
            if len(result[key]) != 6:
                raise ValueError(f"{key} must contain six values")
        directions = [int(value) for value in result["motor_directions"]]
        if any(value not in (-1, 1) for value in directions):
            raise ValueError("motor_directions values must be +1 or -1")
        offsets = [float(value) for value in result["motor_zero_offset_deg"]]
        if any(not math.isfinite(value) for value in offsets):
            raise ValueError("motor_zero_offset_deg values must be finite numbers")
        speed_limits = [int(value) for value in result["motor_speed_limit_dps"]]
        if any(value < 1 or value > 0xFFFF for value in speed_limits):
            raise ValueError("motor_speed_limit_dps values must be in 1..65535")
        matrix = result["joint_to_motor"]
        if len(matrix) != 6 or any(len(row) != 6 for row in matrix):
            raise ValueError("joint_to_motor must be a 6 by 6 matrix")
        if any(not math.isfinite(float(value)) for row in matrix for value in row):
            raise ValueError("joint_to_motor values must be finite numbers")
        # Older user hardware.json files predate limits.  Load the bundled
        # defaults rather than silently disabling this safety check; the UI
        # persists this validated field when the configuration is applied.
        if "joint_position_limits_deg" not in result:
            try:
                with Path(__file__).with_name("default_hardware.json").open(
                    "r", encoding="utf-8"
                ) as stream:
                    result["joint_position_limits_deg"] = json.load(stream)["joint_position_limits_deg"]
            except (OSError, ValueError, KeyError) as error:
                raise ValueError("missing joint_position_limits_deg and bundled defaults unavailable") from error
        limits = result["joint_position_limits_deg"]
        if not isinstance(limits, (list, tuple)) or len(limits) != 6:
            raise ValueError("joint_position_limits_deg must contain six [min, max] pairs")
        checked_limits = []
        for index, pair in enumerate(limits):
            if not isinstance(pair, (list, tuple)) or len(pair) != 2 or any(
                isinstance(value, bool) for value in pair
            ):
                raise ValueError(f"Joint{index + 1} position limits must be a [min, max] pair")
            try:
                minimum, maximum = (float(value) for value in pair)
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError(f"Joint{index + 1} position limits must be finite numbers") from error
            if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum >= maximum:
                raise ValueError(f"Joint{index + 1} position limits must be finite with min < max")
            checked_limits.append([minimum, maximum])
        result["joint_position_limits_deg"] = checked_limits
        result["motor_ids"] = ids
        result["motor_directions"] = directions
        result["motor_zero_offset_deg"] = offsets
        result["motor_speed_limit_dps"] = speed_limits
        result["joint_to_motor"] = [[float(v) for v in row] for row in matrix]
        result["can_interface"] = str(result["can_interface"])
        if not result["can_interface"].strip():
            raise ValueError("can_interface must not be empty")
        enabled = result.setdefault("static_tendon_compensation", False)
        profile = result.setdefault("tendon_profile", "")
        if not isinstance(enabled, bool) or not isinstance(profile, str):
            raise ValueError("static_tendon_compensation must be boolean and tendon_profile must be a path string")
        result["j2_urdf_direction"] = validate_j2_urdf_direction(result.get("j2_urdf_direction", 1))
        return result

    @staticmethod
    def _load_tendon_model(config: dict[str, Any]) -> TendonModel | None:
        if config["static_tendon_compensation"]:
            return TendonModel(
                config["tendon_profile"], j2_urdf_direction=config["j2_urdf_direction"],
            )
        return None

    def close(self) -> None:
        self.bus.close()

    def reconfigure(self, hardware_config: dict[str, Any]) -> None:
        new_config = self._validate_config(hardware_config)
        new_tendon_model = self._load_tendon_model(new_config)
        new_bus = SocketCanBus(new_config["can_interface"], self.timeout_s)
        with self._operation_lock:
            old_bus = self.bus
            self.config = new_config
            self._tendon_model = new_tendon_model
            self._last_tendon_estimate = None
            self.bus = new_bus
            self.motors = [CanMotor(self.bus, motor_id) for motor_id in self.config["motor_ids"]]
            old_bus.close()

    def _map_joint_to_motor(self, joints_deg: Sequence[float]) -> list[float]:
        return joint_to_motor(self.config["joint_to_motor"], joints_deg)

    def _map_motor_to_joint(self, motors_deg: Sequence[float]) -> list[float]:
        return motor_to_joint(self.config["joint_to_motor"], motors_deg)

    def read_positions(self, *, cancel_event: threading.Event | None = None) -> PositionSnapshot:
        with self._operation_lock:
            raw_motor = []
            for motor in self.motors:
                self._check_cancelled(cancel_event)
                raw_motor.append(motor.read_multi_turn_angle_deg())
            self._check_cancelled(cancel_event)
            directions = self.config["motor_directions"]
            offsets = self.config["motor_zero_offset_deg"]
            mapped_motor = [directions[i] * (raw_motor[i] - offsets[i]) for i in range(6)]
            encoder_equivalent = tuple(self._map_motor_to_joint(mapped_motor))
            if self._tendon_model is None:
                return PositionSnapshot(tuple(raw_motor), tuple(mapped_motor), encoder_equivalent)
            estimated = self._tendon_model.estimate(encoder_equivalent, self._last_tendon_estimate)
            self._last_tendon_estimate = estimated
            return PositionSnapshot(tuple(raw_motor), tuple(mapped_motor), estimated, encoder_equivalent)

    def read_status(self) -> list[tuple[Status1, Status2]]:
        with self._operation_lock:
            statuses: list[tuple[Status1, Status2]] = []
            for motor in self.motors:
                statuses.append((motor.read_status1(), motor.read_status2()))
            return statuses

    def _motor_for_index(self, index: int) -> CanMotor:
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(self.motors):
            raise ValueError("motor index must be an integer in 0..5")
        return self.motors[index]

    def read_pid(self, index: int) -> PidGains:
        """Read one physical motor's three-loop raw PI values (0-based index)."""
        with self._operation_lock:
            return self._motor_for_index(index).read_pid()

    def write_pid_persistent(
        self, index: int, gains: PidGains, expected_old: PidGains
    ) -> PidWriteResult:
        """Log intent, write one 0x32, and read back; never retry an uncertain write.

        A readback match checks the current values only, not persistence after reboot.
        The caller must confirm the target motor and safe tuning before invoking this.
        """
        if not isinstance(gains, PidGains) or not isinstance(expected_old, PidGains):
            raise TypeError("new and previously read values must be PidGains")
        with self._operation_lock:
            motor = self._motor_for_index(index)
            before = motor.read_pid()
            if before != expected_old:
                raise RuntimeError("电机当前 PI 与上次读取值不一致；请重新读取并确认，未执行写入")
            if before == gains:
                raise ValueError("新 PI 与当前值相同，未写入 ROM")

            attempt_id = str(uuid4())
            common: dict[str, Any] = {
                "attempt_id": attempt_id,
                "can_interface": self.config["can_interface"],
                "motor_index": index + 1,
                "motor_id": motor.motor_id,
                "before": asdict(before),
                "requested": asdict(gains),
            }
            try:
                _append_pi_history({
                    **common, "event": "intent", "time_utc": datetime.now(timezone.utc).isoformat(),
                })
            except Exception as error:
                raise RuntimeError(f"无法保存 PI 修改意图，未向电机写入：{error}") from error

            write_error: Exception | None = None
            try:
                motor.write_pid_rom(gains)
            except Exception as error:
                # The command may have reached the motor before a reply was lost.
                write_error = error

            observed: PidGains | None = None
            read_error: Exception | None = None
            try:
                observed = motor.read_pid()
            except Exception as error:
                read_error = error

            if write_error is not None:
                status = "write_unconfirmed"
                errors = [f"0x32 未确认：{write_error}"]
                if read_error is not None:
                    errors.append(f"0x30 回读失败：{read_error}")
                error_message = "; ".join(errors)
            elif read_error is not None:
                status = "readback_unconfirmed"
                error_message = f"0x30 回读失败：{read_error}"
            elif observed != gains:
                status = "readback_mismatch"
                error_message = "0x30 回读值与写入值不一致"
            else:
                status = "verified"
                error_message = None

            result = PidWriteResult(motor.motor_id, before, gains, observed, status, error_message)
            try:
                _append_pi_history({
                    **common, "event": "result", "time_utc": datetime.now(timezone.utc).isoformat(),
                    "status": status, "observed": asdict(observed) if observed is not None else None,
                    "error": error_message,
                })
            except Exception as error:
                raise RuntimeError(
                    f"电机 PI 写入可能已经执行，结果记录保存失败；请勿重试，先核验设备：{error}"
                ) from error
            return result

    def motor_on(self) -> None:
        self._broadcast("motor_on")

    def motor_off(self) -> None:
        self._broadcast("motor_off")

    def emergency_stop(self) -> None:
        self._broadcast("emergency_stop")

    def set_current_positions_as_zero(self) -> tuple[MotorZeroResult, ...]:
        """Write each motor's present encoder zero to ROM exactly once.

        Communication errors leave the result unconfirmed: the motor may have
        completed the write before its reply was lost. Software offsets remain
        unchanged and must be independently checked after this operation.
        """
        with self._operation_lock:
            self._last_tendon_estimate = None
            results: list[MotorZeroResult] = []
            for motor in self.motors:
                try:
                    offset = motor.set_current_position_as_zero()
                except Exception as error:
                    results.append(MotorZeroResult(motor.motor_id, None, str(error)))
                else:
                    results.append(MotorZeroResult(motor.motor_id, offset, None))
            return tuple(results)

    def joint_motor_ids(self, joint_index: int) -> tuple[int, ...]:
        """Return IDs of motors affected by a zero-based logical joint index."""
        if isinstance(joint_index, bool) or not isinstance(joint_index, int) or not 0 <= joint_index < 6:
            raise ValueError("joint_index must be an integer in 0..5")
        with self._operation_lock:
            matrix = self.config["joint_to_motor"]
            ids = tuple(self.motors[row].motor_id for row in range(6)
                        if matrix[row][joint_index] != 0.0)
            if not ids:
                raise ValueError(f"joint {joint_index + 1} has no mapped motors")
            return ids

    def joint_motor_on(self, joint_index: int) -> tuple[int, ...]:
        return self._operate_joint_motors(joint_index, "motor_on")

    def joint_motor_off(self, joint_index: int) -> tuple[int, ...]:
        return self._operate_joint_motors(joint_index, "motor_off")

    def _operate_joint_motors(self, joint_index: int, method_name: str) -> tuple[int, ...]:
        with self._operation_lock:
            ids = self.joint_motor_ids(joint_index)
            selected_ids = set(ids)
            selected = [motor for motor in self.motors if motor.motor_id in selected_ids]
            self._operate_motors(selected, method_name)
            return ids

    def _broadcast(self, method_name: str) -> None:
        with self._operation_lock:
            self._operate_motors(self.motors, method_name)

    @staticmethod
    def _operate_motors(motors: Sequence[CanMotor], method_name: str) -> None:
        confirmed: list[int] = []
        errors: list[str] = []
        for motor in motors:
            try:
                getattr(motor, method_name)()
            except Exception as error:  # Continue even if a motor is offline.
                errors.append(f"motor {motor.motor_id}: {error}")
            else:
                confirmed.append(motor.motor_id)
        if errors:
            raise RuntimeError(f"已确认电机 ID: {confirmed}; 未确认: {'; '.join(errors)}")

    def move_increment(self, delta_joint_deg: Sequence[float], joint_speed_limit_dps: float) -> MoveResult:
        if len(delta_joint_deg) != 6:
            raise ValueError("six joint increments are required")
        with self._operation_lock:
            current = self.read_positions()
            target = [current.joint_deg[i] + float(delta_joint_deg[i]) for i in range(6)]
            return self._move_from_snapshot(current, target, joint_speed_limit_dps)

    def move_absolute(self, target_joint_deg: Sequence[float], joint_speed_limit_dps: float) -> MoveResult:
        if len(target_joint_deg) != 6:
            raise ValueError("six joint targets are required")
        with self._operation_lock:
            current = self.read_positions()
            return self._move_from_snapshot(current, target_joint_deg, joint_speed_limit_dps)

    def _check_joint_position_limits(self, joint_deg: Sequence[float], label: str) -> None:
        check_position_limits(joint_deg, self.config["joint_position_limits_deg"], label)

    @staticmethod
    def _check_cancelled(cancel_event: threading.Event | None) -> None:
        if cancel_event is not None and cancel_event.is_set():
            raise InterruptedError("位置操作已取消")

    def check_absolute_target(self, target_joint_deg: Sequence[float]) -> None:
        """Run the exact position-command target checks without reading or writing CAN."""
        with self._operation_lock:
            self._prepare_position_target(target_joint_deg)

    def move_absolute_from_snapshot(
        self,
        current: PositionSnapshot,
        target_joint_deg: Sequence[float],
        joint_speed_limit_dps: float,
        *,
        cancel_event: threading.Event | None = None,
    ) -> MoveResult:
        """Send a position target using a recent feedback sample supplied by the caller."""
        with self._operation_lock:
            return self._move_from_snapshot(
                current, target_joint_deg, joint_speed_limit_dps, cancel_event=cancel_event
            )

    def _prepare_position_target(self, target_joint_deg: Sequence[float]) -> _PreparedTarget:
        if len(target_joint_deg) != 6:
            raise ValueError("six joint targets are required")
        target_joint = tuple(float(value) for value in target_joint_deg)
        if any(not math.isfinite(value) for value in target_joint):
            raise ValueError("joint targets must be finite numbers")
        self._check_joint_position_limits(target_joint, "目标位置")

        tendon = None
        motor_compensation = None
        predicted_joint = None
        effective_joint = target_joint
        if self._tendon_model is not None:
            self._tendon_model.check_limits(target_joint, "目标位置")
            tendon = self._tendon_model.evaluate(target_joint)
            effective_joint = tendon.effective_deg
            self._check_joint_position_limits(effective_joint, "补偿后等效角")
            self._tendon_model.check_limits(effective_joint, "补偿后等效角")
        mapped_target = self._map_joint_to_motor(effective_joint)
        directions = self.config["motor_directions"]
        offsets = self.config["motor_zero_offset_deg"]
        raw_target = [directions[i] * mapped_target[i] + offsets[i] for i in range(6)]
        # Both absolute and increment UI operations send A4 absolute targets.
        # Anchoring an increment to the snapshot prevents a later A8 relative
        # command from shifting the endpoint if the motor moves before receipt.
        # Check the exact quantized A4 target before any motor is written.
        encoded_raw_target = quantize_a4(raw_target)
        encoded_mapped_target = [
            directions[i] * (encoded_raw_target[i] - offsets[i]) for i in range(6)
        ]
        encoded_joint_target = self._map_motor_to_joint(encoded_mapped_target)
        self._check_joint_position_limits(encoded_joint_target, "量化后目标位置")
        joint_target_for_speed = target_joint
        motor_target_for_speed = mapped_target
        if tendon is not None:
            self._tendon_model.check_limits(
                encoded_joint_target, "量化后等效角", ENCODER_LIMIT_TOLERANCE_DEG
            )
            predicted_joint = self._tendon_model.estimate(encoded_joint_target, target_joint)
            self._check_joint_position_limits(predicted_joint, "量化后反馈推算值")
            joint_target_for_speed = predicted_joint
            motor_target_for_speed = encoded_mapped_target
            motor_compensation = tuple(
                direction * value for direction, value in zip(
                    directions, self._map_joint_to_motor(tendon.compensation_deg)
                )
            )
            raw_target = encoded_raw_target

        return _PreparedTarget(
            target_joint, tuple(raw_target), tuple(joint_target_for_speed), tuple(motor_target_for_speed),
            tendon, motor_compensation, predicted_joint,
        )

    def _move_from_snapshot(
        self,
        current: PositionSnapshot,
        target_joint_deg: Sequence[float],
        joint_speed_limit_dps: float,
        *,
        cancel_event: threading.Event | None = None,
    ) -> MoveResult:
        self._check_cancelled(cancel_event)
        speed_limit = float(joint_speed_limit_dps)
        if not math.isfinite(speed_limit) or speed_limit <= 0:
            raise ValueError("joint speed upper limit must be greater than zero degrees/second")
        if any(not math.isfinite(value) for value in current.joint_deg + current.mapped_motor_deg + current.raw_motor_deg):
            raise ValueError("当前位置反馈必须是有限数值，未发送位置命令")
        target = self._prepare_position_target(target_joint_deg)
        delta_joint = [target.speed_joint_deg[i] - current.joint_deg[i] for i in range(6)]
        delta_motor = [target.speed_motor_deg[i] - current.mapped_motor_deg[i] for i in range(6)]

        max_joint_delta = max(abs(value) for value in delta_joint)
        requested_duration_s = max_joint_delta / speed_limit if max_joint_delta > 1e-9 else 0.0
        duration_s = requested_duration_s

        motor_limits = self.config["motor_speed_limit_dps"]
        for index, delta in enumerate(delta_motor):
            if abs(delta) > 1e-9:
                duration_s = max(duration_s, abs(delta) / motor_limits[index])

        if duration_s <= 1e-9:
            duration_s = 1e-9
        requested_motor_speed = [
            abs(delta) / max(requested_duration_s, 1e-9) for delta in delta_motor
        ]
        limited_motor_speed = [abs(delta) / duration_s for delta in delta_motor]
        applied_speed = [
            max(1, min(motor_limits[i], int(math.floor(limited_motor_speed[i] + 1e-9))))
            for i in range(6)
        ]
        attempted = False
        try:
            for i, motor in enumerate(self.motors):
                self._check_cancelled(cancel_event)
                attempted = True
                motor.set_absolute_position_deg(target.raw_motor_deg[i], applied_speed[i])
        except Exception:
            if attempted:
                try:
                    self.emergency_stop()
                except Exception:
                    pass
            raise

        return MoveResult(
            current_joint_deg=tuple(current.joint_deg),
            target_joint_deg=target.joint_deg,
            target_raw_motor_deg=target.raw_motor_deg,
            requested_motor_speed_dps=tuple(requested_motor_speed),
            applied_motor_speed_dps=tuple(applied_speed),
            duration_s=duration_s,
            tendon=target.tendon,
            motor_compensation_deg=target.motor_compensation_deg,
            predicted_joint_deg=target.predicted_joint_deg,
        )
