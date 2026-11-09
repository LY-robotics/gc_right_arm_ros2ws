"""Exercise PI panel callbacks without a Tk display or any CAN interface."""
from __future__ import annotations

from pathlib import Path
from contextlib import ExitStack
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sdk_ui.arm_sdk import PidWriteResult
from sdk_ui.pi_panel import PiPanel
from sdk_ui.protocol import PidGains


class Variable:
    def __init__(self, value: str = "") -> None:
        self.value = value

    def get(self) -> str:
        return self.value

    def set(self, value: str) -> None:
        self.value = value


class PiPanelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sdk = Mock()
        self.sdk.config = {"can_interface": "can2", "motor_ids": [1, 2, 3, 4, 5, 6]}
        self.sdk.read_pid.return_value = PidGains(1, 2, 3, 4, 5, 6)
        self.owner = SimpleNamespace(
            root=object(), busy=False, _ensure_sdk=Mock(return_value=self.sdk),
            _append_status=Mock(), message_var=Mock(),
        )

        def immediate_worker(description, operation, on_success):
            on_success(operation())
        self.owner._start_worker = immediate_worker
        self.panel = PiPanel.__new__(PiPanel)
        self.panel.owner = self.owner
        self.panel.window = object()
        self.panel.baselines = [None] * 6
        self.panel.baseline_devices = [None] * 6
        self.panel.values = [[Variable() for _ in range(6)] for _ in range(6)]
        self.panel.read_values = [[Variable("—") for _ in range(6)] for _ in range(6)]
        self.panel.motor_labels = [Mock() for _ in range(6)]
        self.panel.status_labels = [Mock() for _ in range(6)]

    def test_ui_builds_separate_gray_readonly_and_editable_controls(self) -> None:
        # Build the widget tree using fakes so the assertion does not require X11.
        with ExitStack() as stack:
            for name in ("Toplevel", "Canvas"):
                stack.enter_context(patch(f"sdk_ui.pi_panel.tk.{name}", return_value=Mock()))
            stack.enter_context(patch("sdk_ui.pi_panel.tk.StringVar", side_effect=Variable))
            readonly = stack.enter_context(patch("sdk_ui.pi_panel.tk.Entry", return_value=Mock()))
            editable = stack.enter_context(patch("sdk_ui.pi_panel.ttk.Entry", return_value=Mock()))
            for name in ("Frame", "Scrollbar", "Label", "Button"):
                stack.enter_context(patch(f"sdk_ui.pi_panel.ttk.{name}", return_value=Mock()))
            panel = PiPanel(self.owner)
        self.assertEqual(readonly.call_count, 36)
        self.assertEqual(editable.call_count, 36)
        for call in readonly.call_args_list:
            self.assertEqual(call.kwargs["state"], "readonly")
            self.assertEqual(call.kwargs["readonlybackground"], "#e5e7eb")
        for read, edit in zip(panel.read_values, panel.values):
            self.assertTrue(all(r is not e for r, e in zip(read, edit)))

    def test_readback_and_edit_fields_are_distinct(self) -> None:
        self.panel.read_one(0)
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["1", "2", "3", "4", "5", "6"])
        self.assertEqual([v.get() for v in self.panel.values[0]], ["1", "2", "3", "4", "5", "6"])
        self.panel.values[0][1].set("9")
        self.assertEqual(self.panel.read_values[0][1].get(), "2")
        self.assertEqual(self.panel.baselines[0], PidGains(1, 2, 3, 4, 5, 6))

    def test_connection_change_clears_all_readbacks_but_keeps_edits(self) -> None:
        self.panel.read_one(0)
        self.panel.read_one(1)
        self.panel.values[0][1].set("99")
        self.panel.invalidate_all()
        self.assertEqual(self.panel.baselines, [None] * 6)
        self.assertEqual(self.panel.baseline_devices, [None] * 6)
        self.assertTrue(all(v.get() == "—" for row in self.panel.read_values for v in row))
        self.assertEqual(self.panel.values[0][1].get(), "99")

    def test_reread_refreshes_both_fields(self) -> None:
        self.panel.read_one(0)
        self.panel.values[0][1].set("9")
        self.sdk.read_pid.return_value = PidGains(7, 8, 9, 10, 11, 12)
        self.panel.read_one(0)
        expected = ["7", "8", "9", "10", "11", "12"]
        self.assertEqual([v.get() for v in self.panel.read_values[0]], expected)
        self.assertEqual([v.get() for v in self.panel.values[0]], expected)
        self.assertEqual(self.panel.baselines[0], PidGains(7, 8, 9, 10, 11, 12))

    def test_failed_read_clears_stale_readback_but_preserves_draft(self) -> None:
        self.panel.read_one(0)
        self.panel.values[0][1].set("9")
        self.sdk.read_pid.side_effect = TimeoutError("offline")
        with patch("sdk_ui.pi_panel.messagebox.showwarning"):
            self.panel.read_one(0)
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["—"] * 6)
        self.assertEqual(self.panel.values[0][1].get(), "9")
        self.assertIsNone(self.panel.baselines[0])
        with patch("sdk_ui.pi_panel.messagebox.showerror"):
            self.panel.write_one(0)
        self.sdk.write_pid_persistent.assert_not_called()

    def test_read_all_preserves_only_successful_baselines(self) -> None:
        def read(index):
            if index == 2:
                raise TimeoutError("offline")
            return PidGains(index, 2, 3, 4, 5, 6)
        self.sdk.read_pid.side_effect = read
        with patch("sdk_ui.pi_panel.messagebox.showwarning") as warning:
            self.panel.read_all()
        self.assertEqual(self.sdk.read_pid.call_count, 6)
        self.assertIsNone(self.panel.baselines[2])
        self.assertIsNone(self.panel.baseline_devices[2])
        self.assertEqual(self.panel.baseline_devices[3], ("can2", 4))
        self.assertEqual(self.panel.values[3][0].get(), "3")
        self.assertEqual(self.panel.read_values[3][0].get(), "3")
        self.assertEqual([v.get() for v in self.panel.read_values[2]], ["—"] * 6)
        warning.assert_called_once()

    def test_verified_write_passes_baseline_and_reports_only_current_readback(self) -> None:
        self.panel.read_one(0)
        before = self.panel.baselines[0]
        self.panel.values[0][1].set("9")
        requested = PidGains(1, 9, 3, 4, 5, 6)
        self.sdk.write_pid_persistent.return_value = PidWriteResult(
            1, before, requested, requested, "verified", None)
        with patch("sdk_ui.pi_panel.messagebox.askyesno", return_value=True):
            self.panel.write_one(0)
        self.sdk.write_pid_persistent.assert_called_once_with(0, requested, before)
        self.assertEqual(self.panel.baselines[0], requested)
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["1", "9", "3", "4", "5", "6"])
        self.assertEqual([v.get() for v in self.panel.values[0]], ["1", "9", "3", "4", "5", "6"])
        summary = self.owner._append_status.call_args.args[0]
        self.assertIn("回读一致", summary)
        self.assertIn("断电持久性需重启后核验", summary)
        self.assertNotIn("持久性已验证", summary)

    def test_changed_connection_cannot_use_old_baseline(self) -> None:
        self.panel.read_one(0)
        self.sdk.config["motor_ids"][0] = 9
        with patch("sdk_ui.pi_panel.messagebox.showerror") as error:
            self.panel.write_one(0)
        self.sdk.write_pid_persistent.assert_not_called()
        self.assertIsNone(self.panel.baselines[0])
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["—"] * 6)
        error.assert_called_once()

    def test_write_failure_clears_baseline_before_io(self) -> None:
        self.panel.read_one(0)
        self.panel.values[0][0].set("7")

        def fail(index, desired, baseline):
            self.assertIsNone(self.panel.baselines[index])
            raise RuntimeError("write result unknown")
        self.sdk.write_pid_persistent.side_effect = fail
        with patch("sdk_ui.pi_panel.messagebox.askyesno", return_value=True):
            with self.assertRaisesRegex(RuntimeError, "unknown"):
                self.panel.write_one(0)
        self.assertIsNone(self.panel.baselines[0])
        self.assertIsNone(self.panel.baseline_devices[0])
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["—"] * 6)
        self.assertEqual(self.panel.values[0][0].get(), "7")

    def test_unconfirmed_result_rejects_blind_retry(self) -> None:
        self.panel.read_one(0)
        before = self.panel.baselines[0]
        self.panel.values[0][0].set("7")
        desired = PidGains(7, 2, 3, 4, 5, 6)
        self.sdk.write_pid_persistent.return_value = PidWriteResult(
            1, before, desired, None, "write_unconfirmed", "timeout")
        with patch("sdk_ui.pi_panel.messagebox.askyesno", return_value=True), \
                patch("sdk_ui.pi_panel.messagebox.showwarning") as warning:
            self.panel.write_one(0)
        self.assertIsNone(self.panel.baselines[0])
        self.assertEqual([v.get() for v in self.panel.read_values[0]], ["—"] * 6)
        self.assertEqual(self.panel.values[0][0].get(), "7")
        warning.assert_called_once()
        with patch("sdk_ui.pi_panel.messagebox.showerror") as error:
            self.panel.write_one(0)
        self.assertEqual(self.sdk.write_pid_persistent.call_count, 1)
        error.assert_called_once()

    def test_invalid_value_does_not_write(self) -> None:
        self.panel.read_one(0)
        self.panel.values[0][0].set("256")
        with patch("sdk_ui.pi_panel.messagebox.showerror") as error:
            self.panel.write_one(0)
        self.sdk.write_pid_persistent.assert_not_called()
        error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
