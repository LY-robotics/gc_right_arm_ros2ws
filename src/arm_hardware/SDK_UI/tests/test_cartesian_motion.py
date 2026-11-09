"""Exercise line execution with the real SDK mapping and simulated encoders."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from sdk_ui.arm_sdk import ArmSdk
from sdk_ui.cartesian_motion import CartesianMotionError, _angular_velocity, execute_cartesian_line
from sdk_ui.kinematics import ArmKinematics


class _Motor:
    def __init__(self, motor_id: int, raw_deg: float) -> None:
        self.motor_id = motor_id
        self.raw_deg = raw_deg
        self.commands: list[tuple[float, int]] = []
        self.stops = 0
        self.raw_error = 0.0
        self.cancel: threading.Event | None = None

    def read_multi_turn_angle_deg(self) -> float:
        return self.raw_deg

    def set_absolute_position_deg(self, angle: float, speed: int) -> None:
        self.commands.append((angle, speed))
        self.raw_deg = round(angle * 100) / 100 + self.raw_error
        if self.cancel is not None:
            self.cancel.set()

    def emergency_stop(self) -> None:
        self.stops += 1


class CartesianMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.joints = (20, -35, 20, 55, -15, 10)
        self.model = ArmKinematics()
        start = self.model.forward(self.joints)
        self.target = tuple(a + b for a, b in zip(start.position_m, (-0.008, 0.004, 0.010)))
        config_file = Path(__file__).resolve().parents[1] / "sdk_ui" / "default_hardware.json"
        config = json.loads(config_file.read_text(encoding="utf-8"))
        self.sdk = ArmSdk.__new__(ArmSdk)
        self.sdk.config = ArmSdk._validate_config(config)
        self.sdk._operation_lock = threading.RLock()
        angles = self.sdk._map_joint_to_motor(self.joints)
        self.sdk.motors = [
            _Motor(motor_id, direction * angle + offset)
            for motor_id, direction, angle, offset in zip(
                config["motor_ids"], config["motor_directions"], angles, config["motor_zero_offset_deg"]
            )
        ]
        self.cancel = threading.Event()
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.trace = Path(directory.name) / "traces" / "motion.csv"
        intervals = patch.multiple("sdk_ui.cartesian_motion", _SAMPLE_PERIOD_S=0, _START_SETTLE_S=0)
        intervals.start()
        self.addCleanup(intervals.stop)

    def _run(self, on_sample=None):
        return execute_cartesian_line(
            self.sdk, self.model, self.target, 0.01, 10, self.trace,
            cancel_event=self.cancel, on_sample=on_sample,
        )

    def _rows(self) -> list[dict[str, str]]:
        with self.trace.open(encoding="utf-8", newline="") as stream:
            return list(csv.DictReader(stream))

    def test_completes_sampled_line_and_logs_measured_pose_and_velocity(self) -> None:
        samples = []
        result = self._run(samples.append)
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.trace_path, self.trace)
        self.assertGreater(result.waypoints_completed, 1)
        self.assertEqual(result.samples, len(samples))
        self.assertIsNone(samples[0].speed_m_s)
        self.assertTrue(any(sample.speed_m_s > 0 for sample in samples[1:]))
        self.assertLess(math.dist(samples[-1].position_m, self.target), 0.001)
        self.assertTrue(all(len(motor.commands) == result.waypoints_completed for motor in self.sdk.motors))
        self.assertTrue(all(motor.stops == 0 for motor in self.sdk.motors))
        rows = self._rows()
        self.assertEqual(rows[0]["frame_id"], "base_link")
        self.assertEqual(rows[0]["linear_speed_m_s"], "")
        self.assertTrue(rows[-2]["angular_speed_rad_s"])
        self.assertEqual(rows[-1]["event"], "completed")

    def test_entire_line_preflight_blocks_all_position_writes_on_late_error(self) -> None:
        with patch.object(self.sdk, "check_absolute_target", side_effect=(None, ValueError("wire limit"))):
            with self.assertRaisesRegex(CartesianMotionError, "wire limit") as caught:
                self._run()
        self.assertEqual(caught.exception.trace_path, self.trace)
        self.assertTrue(all(not motor.commands for motor in self.sdk.motors))
        self.assertEqual(self._rows()[-1]["event"], "failed")

    def test_moving_start_is_rejected_before_first_position_write(self) -> None:
        original = self.sdk.read_positions
        reads = 0

        def read(**options):
            nonlocal reads
            reads += 1
            if reads == 2:
                self.sdk.motors[0].raw_deg += 10.08
            return original(**options)

        with patch.object(self.sdk, "read_positions", side_effect=read):
            with self.assertRaisesRegex(CartesianMotionError, "起点仍在移动"):
                self._run()
        self.assertTrue(all(not motor.commands for motor in self.sdk.motors))

    def test_path_deviation_stops_and_preserves_diagnostic_sample(self) -> None:
        self.sdk.motors[0].raw_error = 1008
        with self.assertRaisesRegex(CartesianMotionError, "偏离目标线段"):
            self._run()
        rows = self._rows()
        self.assertGreater(float(rows[-2]["cross_track_error_m"]), 0.003)
        self.assertEqual(rows[-1]["event"], "failed")
        self.assertTrue(all(motor.stops == 1 for motor in self.sdk.motors))

    def test_cancel_mid_motor_group_sends_no_following_position_writes(self) -> None:
        self.sdk.motors[0].cancel = self.cancel
        result = self._run()
        self.assertEqual(result.status, "cancelled")
        self.assertEqual(result.waypoints_completed, 0)
        self.assertEqual(len(self.sdk.motors[0].commands), 1)
        self.assertTrue(all(not motor.commands for motor in self.sdk.motors[1:]))
        self.assertTrue(all(motor.stops >= 1 for motor in self.sdk.motors))
        self.assertEqual(self._rows()[-1]["event"], "cancelled")

    def test_quaternion_sign_flip_preserves_base_frame_angular_velocity(self) -> None:
        # The absolute rotation changes from +90° about Z to that orientation
        # pre-multiplied by an additional +0.2 rad rotation about base X.
        s = math.sqrt(0.5)
        c, r = math.cos(0.1), math.sin(0.1)
        previous = (0, 0, s, s)
        current = tuple(-x for x in (r*s, -r*s, c*s, c*s))
        velocity = _angular_velocity(previous, current, 0.2)
        for value, expected in zip(velocity, (1, 0, 0)):
            self.assertAlmostEqual(value, expected)


if __name__ == "__main__":
    unittest.main()
