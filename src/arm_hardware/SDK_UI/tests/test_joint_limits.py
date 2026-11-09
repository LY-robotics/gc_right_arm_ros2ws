"""Position limits and target quantization tests without actual CAN."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

from sdk_ui.arm_sdk import ArmSdk
from sdk_ui.app import ArmSdkUi

DEFAULT = json.loads((Path(__file__).resolve().parents[1] / "sdk_ui" / "default_hardware.json").read_text())


class FakeMotor:
    def __init__(self, motor_id: int, raw: float) -> None:
        self.motor_id = motor_id
        self.raw = raw
        self.commands: list[tuple[str, float, int]] = []

    def read_multi_turn_angle_deg(self) -> float:
        return self.raw

    def set_absolute_position_deg(self, value: float, speed: int) -> None:
        self.commands.append(("absolute", value, speed))

    def set_incremental_position_deg(self, value: float, speed: int) -> None:
        self.commands.append(("incremental", value, speed))

    def emergency_stop(self) -> None:
        pass


def make_sdk(current: list[float] | None = None, bounds: list[list[float]] | None = None) -> ArmSdk:
    config = copy.deepcopy(DEFAULT)
    config["joint_position_limits_deg"] = bounds or [[-10.0, 10.0] for _ in range(6)]
    config["motor_directions"] = [1] * 6
    config["motor_zero_offset_deg"] = [0.0] * 6
    sdk = ArmSdk.__new__(ArmSdk)
    sdk.config = ArmSdk._validate_config(config)
    sdk._operation_lock = threading.RLock()
    q = current if current is not None else [0.0] * 6
    motor_angles = sdk._map_joint_to_motor(q)
    sdk.motors = [FakeMotor(i + 1, value) for i, value in enumerate(motor_angles)]
    return sdk


class JointLimitsTest(unittest.TestCase):
    def test_old_config_receives_default_limits_without_mutating_input(self) -> None:
        legacy = copy.deepcopy(DEFAULT)
        legacy.pop("joint_position_limits_deg", None)
        result = ArmSdk._validate_config(legacy)
        self.assertEqual(result["joint_position_limits_deg"], DEFAULT["joint_position_limits_deg"])
        self.assertNotIn("joint_position_limits_deg", legacy)

    def test_invalid_limit_configs_rejected(self) -> None:
        for invalid in ([], [[0, 1]] * 5, [[0, 1]] * 7,
                        [[1, 0]] + [[-1, 1]] * 5,
                        [[float("nan"), 1]] + [[-1, 1]] * 5,
                        [[-1, float("inf")]] + [[-1, 1]] * 5):
            config = copy.deepcopy(DEFAULT)
            config["joint_position_limits_deg"] = invalid
            with self.subTest(invalid=invalid), self.assertRaises((ValueError, TypeError)):
                ArmSdk._validate_config(config)

    def test_j2_urdf_direction_uses_strict_integer_config(self) -> None:
        config = copy.deepcopy(DEFAULT)
        config.pop("j2_urdf_direction", None)
        self.assertEqual(ArmSdk._validate_config(config)["j2_urdf_direction"], 1)
        config["j2_urdf_direction"] = -1
        self.assertEqual(ArmSdk._validate_config(config)["j2_urdf_direction"], -1)
        for invalid in (True, False, 1.0, "-1", 0, 2):
            config["j2_urdf_direction"] = invalid
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "j2_urdf_direction"):
                ArmSdk._validate_config(config)

    def test_absolute_target_every_joint_outside_rejects_before_sending(self) -> None:
        for axis in range(6):
            for value in (-10.001, 10.001):
                sdk = make_sdk()
                target = [0.0] * 6
                target[axis] = value
                with self.subTest(axis=axis, value=value), self.assertRaises(ValueError):
                    sdk.move_absolute(target, 1.0)
                self.assertTrue(all(not motor.commands for motor in sdk.motors))

    def test_absolute_inside_and_boundary_are_accepted(self) -> None:
        sdk = make_sdk()
        result = sdk.move_absolute([10.0, 0, 0, 0, 0, 0], 1.0)
        self.assertEqual(result.target_joint_deg[0], 10.0)
        self.assertTrue(all(len(motor.commands) == 1 for motor in sdk.motors))

    def test_increment_rechecks_hardware_feedback_not_cached(self) -> None:
        sdk = make_sdk(current=[9.0, 0, 0, 0, 0, 0])
        result = sdk.move_increment([1.0, 0, 0, 0, 0, 0], 1.0)
        self.assertAlmostEqual(result.target_joint_deg[0], 10.0)
        self.assertTrue(all(motor.commands[0][0] == "absolute" for motor in sdk.motors))
        self.assertAlmostEqual(sdk.motors[0].commands[0][1], -1008.0)
        for motor in sdk.motors:
            motor.commands.clear()
        # New feedback means this command would exceed the positive boundary.
        sdk.motors[0].raw = -100.8 * 9.5
        with self.assertRaises(ValueError):
            sdk.move_increment([1.0, 0, 0, 0, 0, 0], 1.0)
        self.assertTrue(all(not motor.commands for motor in sdk.motors))

    def test_increment_endpoint_is_snapshot_absolute_even_if_motor_moves_before_send(self) -> None:
        sdk = make_sdk(current=[9.0, 0, 0, 0, 0, 0])
        original = sdk.motors[5].read_multi_turn_angle_deg
        def move_motor_during_read():
            captured = original()
            sdk.motors[0].raw = -100.8 * 9.9
            return captured
        sdk.motors[5].read_multi_turn_angle_deg = move_motor_during_read
        sdk.move_increment([0.5, 0, 0, 0, 0, 0], 1.0)
        self.assertEqual(sdk.motors[0].commands[0][0], "absolute")
        self.assertAlmostEqual(sdk.motors[0].commands[0][1], -100.8 * 9.5)

    def test_position_rounding_cannot_place_target_outside_limit(self) -> None:
        bounds = [[-10.0, 10.0] for _ in range(6)]
        bounds[0] = [0.00003, 10.0]
        target = [0.00003, 0, 0, 0, 0, 0]
        for incremental in (False, True):
            sdk = make_sdk(current=[1.0, 0, 0, 0, 0, 0], bounds=bounds)
            with self.subTest(incremental=incremental), self.assertRaises(ValueError):
                if incremental:
                    sdk.move_increment([target[0] - 1.0, 0, 0, 0, 0, 0], 1.0)
                else:
                    sdk.move_absolute(target, 1.0)
            self.assertTrue(all(not motor.commands for motor in sdk.motors))

    def test_coupled_motor_rounding_cannot_exceed_joint_limit(self) -> None:
        bounds = [[-10.0, 10.0] for _ in range(6)]
        bounds[1] = [1.00003, 10.0]
        for incremental in (False, True):
            sdk = make_sdk(current=[0, 2.0, 0, 0, 0, 0], bounds=bounds)
            with self.subTest(incremental=incremental), self.assertRaises(ValueError):
                if incremental:
                    sdk.move_increment([0, -0.99997, 0, 0, 0, 0], 1.0)
                else:
                    sdk.move_absolute([0, 1.00003, 0, 0, 0, 0], 1.0)
            self.assertTrue(all(not motor.commands for motor in sdk.motors))

    def test_demo_and_zero_ui_cannot_bypass_absolute_limits(self) -> None:
        bounds = [[-10.0, 10.0] for _ in range(6)]
        bounds[0] = [0.1, 10.0]
        sdk = make_sdk(current=[1.0, 0, 0, 0, 0, 0], bounds=bounds)
        ui = ArmSdkUi.__new__(ArmSdkUi)
        ui.speed_var = Mock(get=Mock(return_value="1"))
        ui.demo_vars = {"demo1": [Mock(get=Mock(return_value=value)) for value in
                                  ("11", "0", "0", "0", "0", "0")]}
        ui._ensure_sdk = Mock(return_value=sdk)
        ui._show_move_result = Mock()
        ui._start_worker = lambda description, operation, on_success: on_success(operation())
        with self.assertRaises(ValueError):
            ui._move_demo("demo1")
        with self.assertRaises(ValueError):
            ui._move_zero()
        self.assertTrue(all(not motor.commands for motor in sdk.motors))

    def test_nonfinite_target_is_rejected_without_sending(self) -> None:
        sdk = make_sdk()
        with self.assertRaises(ValueError):
            sdk.move_absolute([math.nan, 0, 0, 0, 0, 0], 1.0)
        self.assertTrue(all(not motor.commands for motor in sdk.motors))


if __name__ == "__main__":
    unittest.main()
