"""Shared motor coordinate transforms for the real SDK and offline UI."""

import math
from typing import Sequence


def joint_to_motor(matrix: Sequence[Sequence[float]], joints: Sequence[float]) -> list[float]:
    if len(joints) != 6:
        raise ValueError("six joint values are required")
    return [sum(coefficient * float(angle) for coefficient, angle in zip(row, joints))
            for row in matrix]


def motor_to_joint(matrix: Sequence[Sequence[float]], motors: Sequence[float]) -> list[float]:
    if len(motors) != 6:
        raise ValueError("six motor values are required")
    augmented = [[*row, float(motor)] for row, motor in zip(matrix, motors)]
    for column in range(6):
        pivot = max(range(column, 6), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("joint_to_motor matrix is singular and cannot be inverted")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(6):
            if row != column:
                factor = augmented[row][column]
                augmented[row] = [value - factor * reference for value, reference in
                                  zip(augmented[row], augmented[column])]
    return [row[6] for row in augmented]


def quantize_a4(raw_motor_deg: Sequence[float], *, ros_rounding: bool = False) -> list[float]:
    """Return the exact motor degrees carried by the 0xA4 centidegree fields."""
    encoded = []
    for index, angle in enumerate(raw_motor_deg, 1):
        centi = angle * 100.0
        if not math.isfinite(centi):
            raise ValueError(f"电机 {index} 位置目标无效；未发送位置命令")
        if ros_rounding:
            field = math.trunc(centi)
            fraction = centi - field
            field += (fraction >= 0.5) - (fraction <= -0.5)
        else:
            field = round(centi)
        if not -(2**31) <= field < 2**31:
            raise ValueError(f"电机 {index} 位置目标超过协议范围；未发送位置命令")
        encoded.append(field / 100.0)
    return encoded


def check_position_limits(
    joint_deg: Sequence[float], limits_deg: Sequence[Sequence[float]], label: str
) -> None:
    if len(joint_deg) != 6:
        raise ValueError("six joint positions are required")
    for index, (value, (minimum, maximum)) in enumerate(zip(joint_deg, limits_deg), 1):
        if not math.isfinite(value):
            raise ValueError(f"Joint{index} {label} must be finite")
        # Tolerate floating-point inverse-mapping noise only.
        if value < minimum - 1e-9 or value > maximum + 1e-9:
            raise ValueError(
                f"Joint{index} {label} {value:.9f}° 超过位置限位 "
                f"[{minimum:.9f}, {maximum:.9f}]°；未发送位置命令"
            )
