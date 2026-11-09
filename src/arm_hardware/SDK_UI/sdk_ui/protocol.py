"""Minimal classic-CAN Muweidu motor protocol over Linux SocketCAN."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import threading
import time
from typing import Sequence


CAN_SFF_MASK = 0x7FF
MOTOR_CAN_BASE = 0x140


class CanProtocolError(RuntimeError):
    """Transport or protocol error with the motor that caused it."""


class SocketCanBus:
    def __init__(self, interface: str, timeout_s: float = 0.2) -> None:
        self.interface = interface
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._socket.bind((interface,))
        except Exception:
            self._socket.close()
            raise

    def close(self) -> None:
        self._socket.close()

    def transact(self, motor_id: int, payload: bytes, timeout_s: float | None = None) -> bytes:
        if len(payload) != 8:
            raise ValueError("CAN motor command payload must contain exactly 8 bytes")
        can_id = MOTOR_CAN_BASE + motor_id
        timeout = self.timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        frame = struct.pack("=IB3x8s", can_id, 8, payload)
        with self._lock:
            try:
                self._socket.send(frame)
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("timed out waiting for CAN reply")
                    self._socket.settimeout(remaining)
                    raw = self._socket.recv(16)
                    reply_id, length, data = struct.unpack("=IB3x8s", raw)
                    if (reply_id & (CAN_SFF_MASK | 0xE0000000)) != can_id or length != 8:
                        continue
                    if data[0] != payload[0]:
                        continue
                    return data
            except (OSError, TimeoutError) as error:
                raise CanProtocolError(
                    f"CAN {self.interface}, motor {motor_id}, command 0x{payload[0]:02X}: {error}"
                ) from error


@dataclass(frozen=True)
class Status1:
    temperature_c: int
    voltage_v: float
    current_a: float
    motor_state: int
    fault_flags: int


@dataclass(frozen=True)
class Status2:
    temperature_c: int
    iq_raw: int
    speed_dps: int
    encoder_raw: int


@dataclass(frozen=True)
class PidGains:
    """Raw uint8 PI values, in the exact order of the motor CAN protocol."""

    position_kp: int
    position_ki: int
    speed_kp: int
    speed_ki: int
    torque_kp: int
    torque_ki: int

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
                raise ValueError(f"{name} must be an integer in 0..255")

    def to_bytes(self) -> bytes:
        return bytes((self.position_kp, self.position_ki, self.speed_kp,
                      self.speed_ki, self.torque_kp, self.torque_ki))


class CanMotor:
    def __init__(self, bus: SocketCanBus, motor_id: int) -> None:
        if motor_id < 1 or motor_id > 32:
            raise ValueError(f"motor ID must be 1..32, got {motor_id}")
        self.bus = bus
        self.motor_id = motor_id

    def _command(self, command: int, payload: bytes | None = None) -> bytes:
        data = bytes([command]) + (payload or bytes(7))
        try:
            return self.bus.transact(self.motor_id, data)
        except CanProtocolError as error:
            raise CanProtocolError(str(error)) from error

    @staticmethod
    def _i16(data: bytes, offset: int) -> int:
        return int.from_bytes(data[offset : offset + 2], "little", signed=True)

    def read_status1(self) -> Status1:
        data = self._command(0x9A)
        return Status1(
            temperature_c=int.from_bytes(data[1:2], "little", signed=True),
            voltage_v=int.from_bytes(data[2:4], "little") * 0.01,
            current_a=int.from_bytes(data[4:6], "little") * 0.01,
            motor_state=data[6],
            fault_flags=data[7],
        )

    def read_status2(self) -> Status2:
        data = self._command(0x9C)
        return Status2(
            temperature_c=int.from_bytes(data[1:2], "little", signed=True),
            iq_raw=self._i16(data, 2),
            speed_dps=self._i16(data, 4),
            encoder_raw=int.from_bytes(data[6:8], "little"),
        )

    def read_multi_turn_angle_deg(self) -> float:
        data = self._command(0x92)
        centi_degrees = int.from_bytes(data[1:8], "little", signed=True)
        return centi_degrees * 0.01

    def read_pid(self) -> PidGains:
        data = self._command(0x30)
        return PidGains(*data[2:8])

    def write_pid_rom(self, gains: PidGains) -> None:
        """Send one 0x32 write. A lost or mismatched reply leaves outcome unknown."""
        if not isinstance(gains, PidGains):
            raise TypeError("gains must be PidGains")
        data = self._command(0x32, bytes(1) + gains.to_bytes())
        if data[2:8] != gains.to_bytes():
            raise CanProtocolError(
                f"motor {self.motor_id}: 0x32 reply parameters do not match the request"
            )

    def motor_on(self) -> None:
        self._command(0x88)

    def motor_off(self) -> None:
        self._command(0x80)

    def emergency_stop(self) -> None:
        self._command(0x81)

    def set_current_position_as_zero(self) -> int:
        """Write the present encoder position as the motor's ROM zero point.

        A timeout does not prove that the write failed; do not retry blindly.
        """
        data = self._command(0x19)
        return int.from_bytes(data[6:8], "little")

    def set_absolute_position_deg(self, angle_deg: float, speed_dps: int) -> None:
        if speed_dps < 1 or speed_dps > 0xFFFF:
            raise ValueError("motor speed must be in 1..65535 degrees/second")
        centi_degrees = round(angle_deg * 100.0)
        if centi_degrees < -(2**31) or centi_degrees > 2**31 - 1:
            raise ValueError("absolute motor target exceeds the signed 32-bit 0.01-degree range")
        payload = bytes(1) + int(speed_dps).to_bytes(2, "little") + int(
            centi_degrees
        ).to_bytes(4, "little", signed=True)
        self._command(0xA4, payload)

    def set_incremental_position_deg(self, increment_deg: float, speed_dps: int) -> None:
        if speed_dps < 1 or speed_dps > 0xFFFF:
            raise ValueError("motor speed must be in 1..65535 degrees/second")
        increment_centi_degrees = round(increment_deg * 100.0)
        if increment_centi_degrees < -(2**31) or increment_centi_degrees > 2**31 - 1:
            raise ValueError("increment exceeds the signed 32-bit 0.01-degree range")
        payload = bytes(1) + int(speed_dps).to_bytes(2, "little") + int(
            increment_centi_degrees
        ).to_bytes(4, "little", signed=True)
        self._command(0xA8, payload)
