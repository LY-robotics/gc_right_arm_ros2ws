"""Tests for the new controls without a CAN interface or a Tk display."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sdk_ui import app
from sdk_ui.arm_sdk import ArmSdk, PositionSnapshot
from sdk_ui.protocol import CanMotor


class FakeMotor:
    def __init__(self, motor_id: int, fail: tuple[str, ...] = ()) -> None:
        self.motor_id = motor_id
        self.fail = fail
        self.calls: list[str] = []

    def _call(self, name: str) -> None:
        self.calls.append(name)
        if name in self.fail:
            raise OSError(f"{name} failed")

    def set_current_position_as_zero(self) -> int:
        self._call("zero")
        return self.motor_id * 10

    def motor_on(self) -> None:
        self._call("motor_on")

    def motor_off(self) -> None:
        self._call("motor_off")


def fake_sdk() -> ArmSdk:
    sdk = ArmSdk.__new__(ArmSdk)
    config_path = Path(__file__).resolve().parents[1] / "sdk_ui" / "default_hardware.json"
    sdk.config = ArmSdk._validate_config(json.loads(config_path.read_text(encoding="utf-8")))
    sdk.motors = [FakeMotor(motor_id) for motor_id in sdk.config["motor_ids"]]
    sdk._operation_lock = threading.RLock()
    return sdk


class ProtocolAndSdkControlsTest(unittest.TestCase):
    def test_zero_command_and_little_endian_reply(self) -> None:
        bus = Mock()
        bus.transact.return_value = bytes([0x19, 0, 0, 0, 0, 0, 0x34, 0x12])
        motor = CanMotor(bus, 4)
        self.assertEqual(motor.set_current_position_as_zero(), 0x1234)
        bus.transact.assert_called_once_with(4, bytes([0x19]) + bytes(7))

    def test_zero_attempts_every_motor_once_and_reports_unconfirmed(self) -> None:
        sdk = fake_sdk()
        sdk.motors[1].fail = ("zero",)
        sdk.motors[4].fail = ("zero",)
        before = copy.deepcopy(sdk.config)
        results = sdk.set_current_positions_as_zero()
        self.assertEqual([result.motor_id for result in results], [1, 2, 3, 4, 5, 6])
        self.assertEqual([result.encoder_zero_offset for result in results], [10, None, 30, 40, None, 60])
        self.assertIsNotNone(results[1].error)
        self.assertIsNotNone(results[4].error)
        self.assertTrue(all(motor.calls == ["zero"] for motor in sdk.motors))
        self.assertEqual(sdk.config, before)

    def test_joint_buttons_follow_matrix_including_shared_motors(self) -> None:
        sdk = fake_sdk()
        expected = [(1,), (2, 3), (2, 3), (4, 5), (5,), (6,)]
        self.assertEqual([sdk.joint_motor_ids(i) for i in range(6)], expected)
        self.assertEqual(sdk.joint_motor_on(3), (4, 5))
        self.assertEqual([motor.calls for motor in sdk.motors], [[], [], [], ["motor_on"], ["motor_on"], []])
        self.assertEqual(sdk.joint_motor_off(1), (2, 3))
        self.assertEqual([motor.calls for motor in sdk.motors], [[], ["motor_off"], ["motor_off"], ["motor_on"], ["motor_on"], []])
        for invalid in (-1, 6, True, 1.2):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                sdk.joint_motor_ids(invalid)

    def test_joint_partial_failure_continues_and_reports_each_result(self) -> None:
        sdk = fake_sdk()
        sdk.motors[1].fail = ("motor_on",)
        with self.assertRaises(RuntimeError) as error:
            sdk.joint_motor_on(1)
        self.assertIn("motor 2", str(error.exception))
        self.assertIn("3", str(error.exception))
        self.assertEqual(sdk.motors[1].calls, ["motor_on"])
        self.assertEqual(sdk.motors[2].calls, ["motor_on"])
        self.assertTrue(all(not sdk.motors[i].calls for i in (0, 3, 4, 5)))


class DemoCaptureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "demo_positions.json"
        self.patcher = patch.object(app, "DEMO_CONFIG_PATH", self.path)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.ui = app.ArmSdkUi.__new__(app.ArmSdkUi)
        self.ui.demos = app._default_demos()
        self.ui._update_demo_fields = Mock()
        self.ui._show_snapshot = Mock()
        self.ui._append_status = Mock()
        self.snapshot = PositionSnapshot((10.,) * 6, (20.,) * 6, (1., -2., 3., -4., 5., -6.))
        app._write_json(self.path, self.ui.demos)

    def test_capture_reads_and_persists_corresponding_demo(self) -> None:
        sdk = Mock()
        sdk.read_positions.return_value = self.snapshot
        self.ui._ensure_sdk = Mock(return_value=sdk)
        self.ui._start_worker = lambda description, operation, success: success(operation())
        self.ui._capture_demo("demo3")
        sdk.read_positions.assert_called_once_with()
        self.assertEqual(self.ui.demos["demo3"], list(self.snapshot.joint_deg))
        self.assertEqual(app._read_json(self.path), self.ui.demos)
        self.assertEqual(self.ui.demos["demo0"], [0.] * 6)
        self.ui._update_demo_fields.assert_called_once_with("demo3")
        self.ui._show_snapshot.assert_called_once_with(self.snapshot)

    def test_failed_write_preserves_saved_and_displayed_data(self) -> None:
        before = copy.deepcopy(self.ui.demos)
        with patch.object(Path, "replace", side_effect=OSError("disk error")):
            with self.assertRaisesRegex(OSError, "disk error"):
                self.ui._capture_demo_finished("demo2", self.snapshot)
        self.assertEqual(self.ui.demos, before)
        self.assertEqual(app._read_json(self.path), before)
        self.ui._update_demo_fields.assert_not_called()
        self.ui._show_snapshot.assert_not_called()


if __name__ == "__main__":
    unittest.main()
