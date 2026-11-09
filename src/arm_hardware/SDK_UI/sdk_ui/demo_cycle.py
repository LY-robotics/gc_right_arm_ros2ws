"""Feedback-driven zero/demo alternation, independent of Tk and CAN."""

from __future__ import annotations

import math
from typing import Sequence


class DemoCycle:
    def __init__(
        self, demo_target: Sequence[float], speed_dps: float,
        tolerance_deg: float = 0.1, settle_samples: int = 2,
    ) -> None:
        self.demo_target = self._angles(demo_target)
        self.speed_dps = float(speed_dps)
        self.tolerance_deg = float(tolerance_deg)
        if not math.isfinite(self.speed_dps) or self.speed_dps <= 0:
            raise ValueError("循环关节速度必须是有限正数")
        if not math.isfinite(self.tolerance_deg) or self.tolerance_deg <= 0:
            raise ValueError("循环到位误差必须是有限正数")
        if isinstance(settle_samples, bool) or not isinstance(settle_samples, int) or settle_samples < 1:
            raise ValueError("到位连续采样次数必须是正整数")
        if max(abs(value) for value in self.demo_target) <= self.tolerance_deg:
            raise ValueError("Demo 与逻辑零位过近，请设置大于到位误差的目标")
        self.settle_samples = settle_samples
        self.to_demo = False  # Always establish zero before the first trip.
        self.moving = False
        self.legs_completed = 0
        self.round_trips = 0
        self.error_deg = 0.0
        self._settled = 0
        self._deadline = 0.0

    @staticmethod
    def _angles(values: Sequence[float]) -> tuple[float, ...]:
        if len(values) != 6:
            raise ValueError("循环位置需要六个关节角")
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("循环位置必须是有限数值")
        return result

    @property
    def target(self) -> tuple[float, ...]:
        return self.demo_target if self.to_demo else (0.0,) * 6

    def begin_leg(self, estimated_duration_s: float, now: float) -> None:
        if self.moving:
            raise RuntimeError("上一段循环尚未确认到位")
        if not math.isfinite(estimated_duration_s) or estimated_duration_s < 0 or not math.isfinite(now):
            raise ValueError("循环运动时长无效")
        self._deadline = now + max(10.0, 3.0 * estimated_duration_s + 5.0)
        self._settled = 0
        self.moving = True

    def observe(self, positions_deg: Sequence[float], now: float) -> bool:
        """Return True exactly once when feedback allows the next leg."""
        positions = self._angles(positions_deg)
        if not math.isfinite(now):
            raise ValueError("循环采样时间无效")
        if not self.moving:
            return False
        if now >= self._deadline:
            raise TimeoutError("循环运动到位超时，已停止后续往返")
        self.error_deg = max(abs(a - b) for a, b in zip(positions, self.target))
        self._settled = self._settled + 1 if self.error_deg <= self.tolerance_deg else 0
        if self._settled < self.settle_samples:
            return False
        self.moving = False
        self.legs_completed += 1
        self.round_trips = max(0, (self.legs_completed - 1) // 2)
        self.to_demo = not self.to_demo
        return True
