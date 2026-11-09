"""Verify CAN setup without running ip or asking for authorization."""
from __future__ import annotations

from pathlib import Path
import subprocess
import unittest
from unittest.mock import Mock, patch

from sdk_ui import can_setup


class CanInterfaceValidationTests(unittest.TestCase):
    def test_valid_interface_and_injection_rejected(self) -> None:
        self.assertEqual(can_setup.validate_interface("can2"), "can2")
        for name in ("can0;id", "../can0", "can 0", "-x", "can0\n", "a" * 16, ""):
            with self.subTest(name=name), self.assertRaises(ValueError):
                can_setup.validate_interface(name)

    def test_interface_type_must_be_can(self) -> None:
        with patch.object(Path, "read_text", return_value="280\n"):
            can_setup._check_can_interface("can2")
        with patch.object(Path, "read_text", return_value="1\n"):
            with self.assertRaisesRegex(RuntimeError, "不是 CAN"):
                can_setup._check_can_interface("eth0")
        with patch.object(Path, "read_text", side_effect=FileNotFoundError()):
            with self.assertRaisesRegex(RuntimeError, "不存在"):
                can_setup._check_can_interface("can2")

    def test_invalid_or_non_can_does_not_execute_ip(self) -> None:
        runner = Mock()
        with self.assertRaises(ValueError):
            can_setup.enable_can("can2;id", runner)
        with patch.object(Path, "read_text", return_value="1\n"):
            with self.assertRaises(RuntimeError):
                can_setup.enable_can("eth0", runner)
        runner.assert_not_called()


class CanEnableTests(unittest.TestCase):
    def setUp(self) -> None:
        for name, value in (("_check_can_interface", None), ("_find_ip", "/usr/sbin/ip")):
            patcher = patch.object(can_setup, name, return_value=value)
            patcher.start()
            self.addCleanup(patcher.stop)
        uid = patch.object(can_setup.os, "geteuid", return_value=1000)
        uid.start()
        self.addCleanup(uid.stop)
        system_files = patch.object(Path, "is_file", return_value=True)
        system_files.start()
        self.addCleanup(system_files.stop)
        accessible = patch.object(can_setup.os, "access", return_value=True)
        accessible.start()
        self.addCleanup(accessible.stop)

    def test_three_fixed_commands_use_no_shell(self) -> None:
        runner = Mock(return_value=subprocess.CompletedProcess([], 0, "", ""))
        self.assertIn("can2", can_setup.enable_can("can2", runner))
        commands = [call.args[0] for call in runner.call_args_list]
        self.assertEqual(commands, [
            ["/usr/bin/pkexec", "/usr/sbin/ip", "link", "set", "dev", "can2", "down"],
            ["/usr/bin/pkexec", "/usr/sbin/ip", "link", "set", "dev", "can2", "type", "can", "bitrate", "1000000"],
            ["/usr/bin/pkexec", "/usr/sbin/ip", "link", "set", "dev", "can2", "up"],
        ])
        for call in runner.call_args_list:
            self.assertNotIn("shell", call.kwargs)
            self.assertEqual(call.kwargs["stdin"], subprocess.DEVNULL)

    def test_step_failure_stops_and_reports_interface_state(self) -> None:
        runner = Mock(side_effect=[
            subprocess.CompletedProcess([], 0, "", ""),
            subprocess.CompletedProcess([], 1, "", "authorization denied"),
        ])
        with self.assertRaises(RuntimeError) as error:
            can_setup.enable_can("can2", runner)
        self.assertIn("关闭接口", str(error.exception))
        self.assertIn("关闭状态", str(error.exception))
        self.assertIn("authorization denied", str(error.exception))
        self.assertEqual(runner.call_count, 2)

    def test_first_step_denied_does_not_try_remaining_steps(self) -> None:
        runner = Mock(return_value=subprocess.CompletedProcess([], 126, "", "permission denied"))
        with self.assertRaises(RuntimeError) as error:
            can_setup.enable_can("can2", runner)
        self.assertIn("无", str(error.exception))
        self.assertIn("状态未知", str(error.exception))
        self.assertEqual(runner.call_count, 1)

    def test_sudo_noninteractive_fallback_and_no_privilege_tool(self) -> None:
        runner = Mock(return_value=subprocess.CompletedProcess([], 0, "", ""))
        def is_file(path: Path) -> bool:
            return path.as_posix() == "/usr/bin/sudo"
        with patch.object(Path, "is_file", is_file):
            can_setup.enable_can("can2", runner)
        self.assertEqual(runner.call_args_list[0].args[0][:3], ["/usr/bin/sudo", "-n", "/usr/sbin/ip"])
        runner.reset_mock()
        with patch.object(Path, "is_file", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "pkexec 或 sudo"):
                can_setup.enable_can("can2", runner)
        runner.assert_not_called()

    def test_timeout_stops_remaining_steps(self) -> None:
        runner = Mock(side_effect=[
            subprocess.CompletedProcess([], 0, "", ""),
            subprocess.TimeoutExpired("pkexec", 120),
        ])
        with self.assertRaises(RuntimeError) as error:
            can_setup.enable_can("can2", runner)
        self.assertIn("关闭状态", str(error.exception))
        self.assertEqual(runner.call_count, 2)


if __name__ == "__main__":
    unittest.main()
