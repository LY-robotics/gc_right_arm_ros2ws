"""Python binding for the shared static tendon model. No CAN or ROS I/O."""

from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from typing import Sequence
import weakref
import xml.etree.ElementTree as ET


_Vector = ct.c_double * 3
_Angles = ct.c_double * 6
_DoublePointer = ct.POINTER(ct.c_double)
_CHAIN = ("world", "jian", "base", "Arm1", "Arm2", "Arm3", "Arm4", "Arm5", "Arm6")
_JOINTS = ("Joint0", "Joint0_1", *(f"Joint{i}" for i in range(1, 7)))
ENCODER_LIMIT_TOLERANCE_DEG = math.degrees(1e-5)


class _Config(ct.Structure):
    _fields_ = [
        ("origin_xyz_m", _Vector * 8),
        ("origin_rpy_rad", _Vector * 8),
        ("axis", _Vector * 6),
        ("lower_rad", _Angles),
        ("upper_rad", _Angles),
        ("mass_kg", _Angles),
        ("com_m", _Vector * 6),
        ("rope_stiffness_left_n_per_mm", _Angles),
        ("rope_stiffness_right_n_per_mm", _Angles),
        ("radius_mm", _Angles),
        ("gravity_m_s2", _Vector),
        ("payload_mass_kg", ct.c_double),
        ("payload_com_m", _Vector),
    ]


class _Result(ct.Structure):
    _fields_ = [(name, _Angles) for name in (
        "hold_torque_nm", "compensation_rad", "effective_rad", "rope_a_delta_mm", "rope_a_delta_n",
        "rope_b_delta_n",
    )]


@dataclass(frozen=True)
class TendonEvaluation:
    hold_torque_nm: tuple[float, ...]
    compensation_deg: tuple[float, ...]
    effective_deg: tuple[float, ...]
    rope_a_delta_mm: tuple[float, ...]
    rope_a_delta_n: tuple[float, ...]
    rope_b_delta_n: tuple[float, ...]


def _prefixes() -> list[Path]:
    return [Path(value) for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if value]


def package_file(package: str, relative: str) -> Path:
    """Find a resource next to the source/install tree, then on the ROS overlay."""
    locations = [parent / package for parent in Path(__file__).resolve().parents]
    locations.extend(prefix / "share" / package for prefix in _prefixes())
    for location in locations:
        candidate = location / relative
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"未找到 {package}/{relative}。请使用项目源码或 source ROS 工作区。")


def _checked_vector(values: Sequence[float], size: int, label: str) -> list[float]:
    if isinstance(values, (str, bytes)) or len(values) != size:
        raise ValueError(f"{label} 必须包含 {size} 个数值")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} 必须是有限数值")
    return result


def _load_config(profile: Path, urdf: Path) -> _Config:
    with profile.open(encoding="utf-8") as stream:
        parameters = json.load(stream)
    config = _Config()
    for name, length in (
        ("mass_kg", 6), ("radius_mm", 6),
        ("gravity_m_s2", 3), ("payload_com_m", 3),
    ):
        getattr(config, name)[:] = _checked_vector(parameters[name], length, name)
    left_name = "rope_stiffness_left_n_per_mm"
    right_name = "rope_stiffness_right_n_per_mm"
    legacy_name = "rope_stiffness_n_per_mm"
    if legacy_name in parameters:
        if left_name in parameters or right_name in parameters:
            raise ValueError("旧版单绳刚度字段与左右绳刚度字段不能混用")
        left = right = _checked_vector(parameters[legacy_name], 6, legacy_name)
    elif left_name in parameters and right_name in parameters:
        left = _checked_vector(parameters[left_name], 6, left_name)
        right = _checked_vector(parameters[right_name], 6, right_name)
    else:
        raise ValueError("需要同时配置左绳和右绳刚度")
    config.rope_stiffness_left_n_per_mm[:] = left
    config.rope_stiffness_right_n_per_mm[:] = right

    config.payload_mass_kg = float(parameters["payload_mass_kg"])
    if len(parameters["com_m"]) != 6:
        raise ValueError("com_m 必须包含六个局部重心")
    for index, center in enumerate(parameters["com_m"]):
        config.com_m[index][:] = _checked_vector(center, 3, f"com_m[{index}]")

    robot = ET.parse(urdf).getroot()
    for index, name in enumerate(_JOINTS):
        joint = robot.find(f"./joint[@name='{name}']")
        joint_type = "fixed" if index < 2 else "revolute"
        if joint is None or joint.get("type") != joint_type:
            raise ValueError(f"URDF 中缺少 {joint_type} 关节 {name}")
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or parent.get("link") != _CHAIN[index]
                or child.get("link") != _CHAIN[index + 1]):
            raise ValueError(f"URDF 中 {name} 的父子连杆与补偿模型不符")
        origin = joint.find("origin")
        if origin is not None:
            for attribute, output in (
                ("xyz", config.origin_xyz_m), ("rpy", config.origin_rpy_rad)
            ):
                output[index][:] = _checked_vector(
                    origin.get(attribute, "0 0 0").split(), 3, f"{name} {attribute}"
                )
        if index >= 2:
            j = index - 2
            axis = joint.find("axis")
            config.axis[j][:] = _checked_vector(
                axis.get("xyz", "1 0 0").split() if axis is not None else ("1", "0", "0"),
                3, f"{name} axis"
            )
            limit = joint.find("limit")
            if limit is None:
                raise ValueError(f"URDF 中缺少 {name} 的位置限位")
            config.lower_rad[j], config.upper_rad[j] = (
                float(limit.attrib[key]) for key in ("lower", "upper")
            )
    return config


def _load_library() -> ct.CDLL:
    override = os.environ.get("ARM_TENDON_LIBRARY")
    if sys.platform == "win32":
        name = "arm_tendon_compensation.dll"
    elif sys.platform == "darwin":
        name = "libarm_tendon_compensation.dylib"
    else:
        name = "libarm_tendon_compensation.so"
    locations = []
    for parent in Path(__file__).resolve().parents:
        locations.extend((parent / ".tendon-build" / name, parent / ".tendon-build" / "Release" / name))
    locations.extend(prefix / subdir / name for prefix in _prefixes() for subdir in ("lib", "bin"))
    path = Path(override).expanduser() if override else next(
        (candidate for candidate in locations if candidate.is_file()), None
    )
    if path is None:
        raise FileNotFoundError(
            f"未找到 {name}。请构建 arm_hardware 并 source install/setup.bash，"
            "或设置 ARM_TENDON_LIBRARY 指向单独构建的库。"
        )
    library = ct.CDLL(str(path.resolve()))
    library.arm_tendon_abi_version.argtypes = []
    library.arm_tendon_abi_version.restype = ct.c_uint
    if library.arm_tendon_abi_version() != 2:
        raise RuntimeError(f"绳驱补偿库版本不匹配，请重新构建 {path}")
    error_args = [ct.c_char_p, ct.c_size_t]
    library.arm_tendon_create.argtypes = [ct.POINTER(_Config), *error_args]
    library.arm_tendon_create.restype = ct.c_void_p
    library.arm_tendon_destroy.argtypes = [ct.c_void_p]
    library.arm_tendon_destroy.restype = None
    library.arm_tendon_evaluate.argtypes = [
        ct.c_void_p, _DoublePointer, ct.POINTER(_Result), *error_args
    ]
    library.arm_tendon_evaluate.restype = ct.c_int
    library.arm_tendon_estimate.argtypes = [ct.c_void_p, *([_DoublePointer] * 3), *error_args]
    library.arm_tendon_estimate.restype = ct.c_int
    library.arm_tendon_coupled_matrix.argtypes = [_DoublePointer]
    library.arm_tendon_coupled_matrix.restype = None
    return library


def validate_j2_urdf_direction(direction: object) -> int:
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("j2_urdf_direction 必须是整数 1 或 -1")
    return direction


class TendonModel:
    def __init__(self, profile: str | Path = "", *, j2_urdf_direction: int = 1) -> None:
        self._j2_urdf_direction = validate_j2_urdf_direction(j2_urdf_direction)
        profile = profile or os.environ.get("ARM_TENDON_PROFILE")
        urdf = os.environ.get("ARM_TENDON_URDF")
        self.profile = Path(profile).expanduser().resolve() if profile else package_file(
            "arm_hardware", "config/static_tendon_compensation.json"
        )
        self.urdf = Path(urdf).expanduser().resolve() if urdf else package_file(
            "arm_description", "urdf/Arm_R.urdf.xacro"
        )
        config = _load_config(self.profile, self.urdf)
        limits = [
            (math.degrees(lower), math.degrees(upper))
            for lower, upper in zip(config.lower_rad, config.upper_rad)
        ]
        if self._j2_urdf_direction == -1:
            lower, upper = limits[1]
            limits[1] = (-upper, -lower)
        self.position_limits_deg = tuple(limits)
        self.library = _load_library()
        error = ct.create_string_buffer(512)
        self._handle = self.library.arm_tendon_create(ct.byref(config), error, len(error))
        if not self._handle:
            raise ValueError(error.value.decode())
        weakref.finalize(self, self.library.arm_tendon_destroy, self._handle)

    def _convert_j2(self, values: Sequence[float]) -> tuple[float, ...]:
        # The same sign transforms SDK to URDF and URDF to SDK.
        return values[0], self._j2_urdf_direction * values[1], *values[2:]

    def _urdf_angles_rad(self, values: Sequence[float]) -> _Angles:
        return _Angles(*map(math.radians, self._convert_j2(_checked_vector(values, 6, "关节角"))))

    def check_limits(
        self, angles_deg: Sequence[float], label: str, tolerance_deg: float = 1e-9
    ) -> None:
        # Allow roundoff from the decimal radian limits in the URDF.
        for index, (value, (lower, upper)) in enumerate(zip(
            _checked_vector(angles_deg, 6, label), self.position_limits_deg
        )):
            if not lower - tolerance_deg <= value <= upper + tolerance_deg:
                raise ValueError(
                    f"J{index + 1} {label} {value:.8f}° 超过 URDF 限位 [{lower:.8f}, {upper:.8f}]°"
                )

    def evaluate(self, nominal_deg: Sequence[float]) -> TendonEvaluation:
        error = ct.create_string_buffer(512)
        result = _Result()
        status = self.library.arm_tendon_evaluate(
            self._handle, self._urdf_angles_rad(nominal_deg), ct.byref(result), error, len(error)
        )
        if status:
            raise ValueError(error.value.decode())
        return TendonEvaluation(
            self._convert_j2(result.hold_torque_nm),
            self._convert_j2(tuple(map(math.degrees, result.compensation_rad))),
            self._convert_j2(tuple(map(math.degrees, result.effective_rad))),
            tuple(result.rope_a_delta_mm),
            tuple(result.rope_a_delta_n),
            tuple(result.rope_b_delta_n),
        )

    def estimate(
        self, encoder_deg: Sequence[float], seed_deg: Sequence[float] | None = None
    ) -> tuple[float, ...]:
        error = ct.create_string_buffer(512)
        output = _Angles()
        status = self.library.arm_tendon_estimate(
            self._handle, self._urdf_angles_rad(encoder_deg),
            self._urdf_angles_rad(seed_deg) if seed_deg is not None else None, output, error, len(error)
        )
        if status:
            raise ValueError(f"静态反馈反解失败：{error.value.decode()}")
        return self._convert_j2(tuple(map(math.degrees, output)))

    def coupled_motor_matrix(self) -> tuple[tuple[float, ...], ...]:
        """Return the native ROS coupling matrix, in URDF joint coordinates."""
        values = (ct.c_double * 36)()
        self.library.arm_tendon_coupled_matrix(values)
        return tuple(tuple(values[i:i + 6]) for i in range(0, 36, 6))
