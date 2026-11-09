"""Motor PI protocol and durable audit behavior without CAN hardware."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_ui import arm_sdk
from sdk_ui.arm_sdk import ArmSdk
from sdk_ui.protocol import CanMotor, CanProtocolError, PidGains


OLD = PidGains(1, 2, 3, 4, 5, 6)
NEW = PidGains(10, 20, 30, 40, 50, 60)
OTHER = PidGains(11, 20, 30, 40, 50, 60)


class PiProtocolTest(unittest.TestCase):
    def test_read_and_write_frames_are_exact(self):
        bus = Mock()
        bus.transact.side_effect = [
            bytes([0x30, 0, 1, 2, 3, 4, 5, 6]),
            bytes([0x32, 0, 10, 20, 30, 40, 50, 60]),
        ]
        motor = CanMotor(bus, 7)
        self.assertEqual(motor.read_pid(), OLD)
        motor.write_pid_rom(NEW)
        self.assertEqual(bus.transact.call_args_list[0].args,
                         (7, bytes([0x30, 0, 0, 0, 0, 0, 0, 0])))
        self.assertEqual(bus.transact.call_args_list[1].args,
                         (7, bytes([0x32, 0, 10, 20, 30, 40, 50, 60])))

    def test_validation_and_mismatched_ack(self):
        for value in (-1, 256, True, 1.5, "1"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                PidGains(value, 0, 0, 0, 0, 0)
        bus = Mock()
        bus.transact.return_value = bytes([0x32, 0, 10, 20, 30, 40, 50, 61])
        with self.assertRaises(CanProtocolError):
            CanMotor(bus, 1).write_pid_rom(NEW)
        with self.assertRaises(TypeError):
            CanMotor(bus, 1).write_pid_rom([10, 20, 30, 40, 50, 60])


class PiHistoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "pi_history.jsonl"
        self.path_patch = patch.object(arm_sdk, "PI_HISTORY_PATH", self.path)
        self.path_patch.start()
        self.addCleanup(self.path_patch.stop)
        self.sdk = ArmSdk.__new__(ArmSdk)
        self.sdk.config = {"can_interface": "can2"}
        self.sdk._operation_lock = threading.RLock()
        self.motor = Mock()
        self.motor.motor_id = 9
        self.sdk.motors = [self.motor]

    def records(self):
        return [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()]

    def test_success_records_intent_before_write_and_current_readback(self):
        events = []
        self.motor.read_pid.side_effect = lambda: (events.append("read") or (OLD if events.count("read") == 1 else NEW))
        self.motor.write_pid_rom.side_effect = lambda gains: events.append("write")
        append = arm_sdk._append_pi_history
        def append_spy(event):
            events.append(event["event"])
            append(event)
        with patch.object(arm_sdk, "_append_pi_history", side_effect=append_spy):
            result = self.sdk.write_pid_persistent(0, NEW, OLD)
        self.assertEqual(events, ["read", "intent", "write", "read", "result"])
        self.assertEqual(result.status, "verified")
        self.assertEqual(result.observed, NEW)
        records = self.records()
        self.assertEqual([record["event"] for record in records], ["intent", "result"])
        self.assertEqual(records[0]["attempt_id"], records[1]["attempt_id"])
        self.assertEqual(records[1]["motor_id"], 9)
        self.assertEqual(records[1]["before"]["position_kp"], 1)
        self.assertEqual(records[1]["requested"]["torque_ki"], 60)
        self.assertEqual(records[1]["observed"]["torque_ki"], 60)
        self.motor.write_pid_rom.assert_called_once_with(NEW)

    def test_lost_write_reply_is_unconfirmed_even_if_readback_matches(self):
        self.motor.read_pid.side_effect = [OLD, NEW]
        self.motor.write_pid_rom.side_effect = TimeoutError("reply lost")
        result = self.sdk.write_pid_persistent(0, NEW, OLD)
        self.assertEqual(result.status, "write_unconfirmed")
        self.assertEqual(result.observed, NEW)
        self.assertIn("reply lost", result.error)
        self.assertEqual(self.records()[1]["status"], "write_unconfirmed")
        self.motor.write_pid_rom.assert_called_once_with(NEW)

    def test_readback_mismatch_and_unavailable(self):
        self.motor.read_pid.side_effect = [OLD, OTHER]
        result = self.sdk.write_pid_persistent(0, NEW, OLD)
        self.assertEqual(result.status, "readback_mismatch")
        self.assertEqual(self.records()[1]["observed"]["position_kp"], 11)
        self.motor.read_pid.side_effect = [OLD, TimeoutError("read failed")]
        result = self.sdk.write_pid_persistent(0, NEW, OLD)
        self.assertEqual(result.status, "readback_unconfirmed")
        self.assertIsNone(result.observed)
        self.assertEqual(self.records()[-1]["status"], "readback_unconfirmed")

    def test_changed_values_and_identical_values_do_not_write(self):
        self.motor.read_pid.return_value = OTHER
        with self.assertRaisesRegex(RuntimeError, "不一致"):
            self.sdk.write_pid_persistent(0, NEW, OLD)
        self.motor.read_pid.return_value = OLD
        with self.assertRaisesRegex(ValueError, "相同"):
            self.sdk.write_pid_persistent(0, OLD, OLD)
        self.assertFalse(self.path.exists())
        self.motor.write_pid_rom.assert_not_called()

    def test_failed_intent_log_prevents_write(self):
        self.motor.read_pid.return_value = OLD
        with patch.object(arm_sdk, "_append_pi_history", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(RuntimeError, "未向电机写入"):
                self.sdk.write_pid_persistent(0, NEW, OLD)
        self.motor.write_pid_rom.assert_not_called()

    def test_partial_intent_record_rolls_back_before_any_motor_write(self):
        self.path.write_text('{"event":"older"}\n', encoding="utf-8")
        self.motor.read_pid.return_value = OLD
        real_write = os.write
        calls = 0
        def partial_then_fail(fd, data):
            nonlocal calls
            calls += 1
            if calls == 1:
                return real_write(fd, data[:7])
            raise OSError("disk full")
        with patch.object(arm_sdk.os, "write", side_effect=partial_then_fail):
            with self.assertRaisesRegex(RuntimeError, "未向电机写入"):
                self.sdk.write_pid_persistent(0, NEW, OLD)
        self.assertEqual(self.path.read_text(encoding="utf-8"), '{"event":"older"}\n')
        self.motor.write_pid_rom.assert_not_called()

    def test_failed_result_log_warns_write_may_have_executed(self):
        self.motor.read_pid.side_effect = [OLD, NEW]
        append = arm_sdk._append_pi_history
        def fail_result(event):
            if event["event"] == "result":
                raise OSError("disk full")
            append(event)
        with patch.object(arm_sdk, "_append_pi_history", side_effect=fail_result):
            with self.assertRaisesRegex(RuntimeError, "可能已经执行"):
                self.sdk.write_pid_persistent(0, NEW, OLD)
        self.motor.write_pid_rom.assert_called_once_with(NEW)
        self.assertEqual([record["event"] for record in self.records()], ["intent"])

    def test_invalid_input_never_touches_motor(self):
        for index in (True, -1, 1, 0.0):
            with self.subTest(index=index), self.assertRaises(ValueError):
                self.sdk.read_pid(index)
        with self.assertRaises(TypeError):
            self.sdk.write_pid_persistent(0, NEW, [1, 2, 3, 4, 5, 6])
        self.motor.read_pid.assert_not_called()
        self.motor.write_pid_rom.assert_not_called()


if __name__ == "__main__":
    unittest.main()
