"""Continuous position polling and action priority without Tk or CAN."""
from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_ui.app import ArmSdkUi, LIVE_READ_INTERVAL_MS
from sdk_ui.arm_sdk import PositionSnapshot


class FakeRoot:
    def __init__(self) -> None:
        self.callbacks = {}
        self.sequence = 0
        self.destroyed = False

    def after(self, delay, callback):
        self.sequence += 1
        key = f"after-{self.sequence}"
        self.callbacks[key] = (delay, callback)
        return key

    def after_cancel(self, key):
        self.callbacks.pop(key, None)

    def run(self, key):
        _, callback = self.callbacks.pop(key)
        callback()

    def destroy(self):
        self.destroyed = True


class ImmediateThread:
    def __init__(self, *, target, daemon):
        self.target = target
        assert daemon

    def start(self):
        self.target()


class LiveReadTest(unittest.TestCase):
    def setUp(self) -> None:
        self.thread_patch = patch("sdk_ui.app.threading.Thread", ImmediateThread)
        self.thread_patch.start()
        self.addCleanup(self.thread_patch.stop)
        self.ui = ArmSdkUi.__new__(ArmSdkUi)
        self.ui.root = FakeRoot()
        self.ui.busy = False
        self.ui.live_read_enabled = False
        self.ui._live_read_inflight = False
        self.ui._live_generation = 0
        self.ui._live_after_id = None
        self.ui._pending_operation = None
        self.ui.live_read_button = Mock()
        self.ui.live_read_status = Mock()
        self.ui.message_var = Mock()
        self.ui._show_snapshot = Mock()
        self.ui._append_status = Mock()
        self.ui.current_labels = [Mock() for _ in range(6)]
        self.ui.motor_position_label = Mock()
        self.ui.sdk = Mock()
        self.snapshot = PositionSnapshot((1.0,) * 6, (2.0,) * 6, (3.0,) * 6)
        self.ui.sdk.read_positions.return_value = self.snapshot
        self.ui.demos = {"demo0": [0.0] * 6}

    def _start_poll(self) -> str:
        self.ui._toggle_live_read()
        timer = self.ui._live_after_id
        self.assertIsNotNone(timer)
        self.ui.root.run(timer)
        self.assertTrue(self.ui.busy)
        self.assertTrue(self.ui._live_read_inflight)
        self.assertEqual(len(self.ui.root.callbacks), 1)
        return next(iter(self.ui.root.callbacks))

    def test_success_updates_display_and_schedules_without_saving_demo(self) -> None:
        callback = self._start_poll()
        self.ui.root.run(callback)
        self.ui._show_snapshot.assert_called_once_with(self.snapshot)
        self.assertEqual(self.ui.demos["demo0"], [0.0] * 6)
        self.assertFalse(self.ui.busy)
        self.assertFalse(self.ui._live_read_inflight)
        timer = self.ui._live_after_id
        self.assertEqual(self.ui.root.callbacks[timer][0], LIVE_READ_INTERVAL_MS)
        self.ui._toggle_live_read()
        self.assertFalse(self.ui.live_read_enabled)
        self.assertEqual(self.ui.root.callbacks, {})

    def test_busy_poll_reschedules_without_reading(self) -> None:
        self.ui.busy = True
        self.ui._toggle_live_read()
        self.ui.root.run(self.ui._live_after_id)
        self.ui.sdk.read_positions.assert_not_called()
        self.assertEqual(len(self.ui.root.callbacks), 1)
        self.assertEqual(self.ui.root.callbacks[self.ui._live_after_id][0], LIVE_READ_INTERVAL_MS)

    def test_stop_ignores_old_inflight_result(self) -> None:
        callback = self._start_poll()
        self.ui._toggle_live_read()
        self.ui.root.run(callback)
        self.ui._show_snapshot.assert_not_called()
        self.assertFalse(self.ui.busy)
        self.assertEqual(self.ui.root.callbacks, {})

    def test_stop_then_restart_keeps_only_new_generation(self) -> None:
        stale_callback = self._start_poll()
        self.ui._toggle_live_read()
        self.ui._toggle_live_read()
        new_timer = self.ui._live_after_id
        self.ui.root.run(stale_callback)
        self.ui._show_snapshot.assert_not_called()
        self.assertEqual(self.ui._live_after_id, new_timer)
        self.ui.root.run(new_timer)
        callback = next(iter(self.ui.root.callbacks))
        self.ui.root.run(callback)
        self.ui._show_snapshot.assert_called_once_with(self.snapshot)
        self.assertEqual(len(self.ui.root.callbacks), 1)

    def test_poll_failure_stops_and_does_not_reschedule(self) -> None:
        self.ui.sdk.read_positions.side_effect = TimeoutError("offline")
        callback = self._start_poll()
        with patch("sdk_ui.app.messagebox.showerror") as error:
            self.ui.root.run(callback)
        self.assertFalse(self.ui.live_read_enabled)
        self.assertFalse(self.ui.busy)
        self.assertEqual(self.ui.root.callbacks, {})
        self.ui._show_snapshot.assert_not_called()
        for label in self.ui.current_labels:
            label.configure.assert_called_once_with(text="—")
        self.ui.motor_position_label.configure.assert_called_once_with(text="电机角度：读取失败，位置未知")
        error.assert_called_once()

    def test_action_during_poll_is_queued_once_then_executed(self) -> None:
        callback = self._start_poll()
        action = Mock()
        self.ui._start_worker("manual", action)
        action.assert_not_called()
        self.ui.root.run(callback)
        action.assert_called_once()
        self.assertTrue(self.ui.busy)
        self.assertIsNone(self.ui._pending_operation)
        self.assertTrue(self.ui.live_read_enabled)
        for key, (delay, _) in list(self.ui.root.callbacks.items()):
            if delay == 0:
                self.ui.root.run(key)
        self.assertFalse(self.ui.busy)

    def test_emergency_overrides_queued_action_before_next_poll(self) -> None:
        callback = self._start_poll()
        movement = Mock()
        stop = Mock()
        self.ui._start_worker("movement", movement)
        self.ui._start_worker("emergency", stop, priority=True)
        self.ui.root.run(callback)
        movement.assert_not_called()
        stop.assert_called_once()
        self.ui._append_status.assert_called_once()
        self.assertIsNone(self.ui._pending_operation)

    def test_emergency_dispatched_before_success_callback(self) -> None:
        events = []
        self.ui._start_worker("normal", lambda: "ok", lambda result: events.append("success"))
        callback = next(iter(self.ui.root.callbacks))
        self.ui._start_worker("emergency", lambda: events.append("stop"), priority=True)
        self.ui.root.run(callback)
        self.assertEqual(events, ["stop", "success"])
        self.assertTrue(self.ui.busy)
        self.ui.root.run(next(iter(self.ui.root.callbacks)))
        self.assertFalse(self.ui.busy)

    def test_error_in_success_callback_does_not_clear_emergency_busy(self) -> None:
        events = []
        def failed_handler(result):
            raise ValueError("display error")
        self.ui._start_worker("normal", lambda: "ok", failed_handler)
        callback = next(iter(self.ui.root.callbacks))
        self.ui._start_worker("emergency", lambda: events.append("stop"), priority=True)
        with patch("sdk_ui.app.messagebox.showerror") as message:
            self.ui.root.run(callback)
        self.assertEqual(events, ["stop"])
        self.assertTrue(self.ui.busy)
        message.assert_called_once()
        self.ui.root.run(next(iter(self.ui.root.callbacks)))
        self.assertFalse(self.ui.busy)

    def test_close_stops_poll_and_waits_for_inflight_before_destroy(self) -> None:
        callback = self._start_poll()
        with patch("sdk_ui.app.messagebox.showinfo") as info:
            self.ui._close()
        info.assert_called_once()
        self.assertFalse(self.ui.live_read_enabled)
        self.assertFalse(self.ui.root.destroyed)
        self.ui.root.run(callback)
        self.ui._show_snapshot.assert_not_called()
        self.ui._close()
        self.assertTrue(self.ui.root.destroyed)
        self.ui.sdk.close.assert_called_once()

    def test_failed_connection_does_not_start_poll(self) -> None:
        self.ui.sdk = None
        with patch("sdk_ui.app.messagebox.showerror") as error:
            self.ui._toggle_live_read()
        self.assertFalse(self.ui.live_read_enabled)
        self.assertEqual(self.ui.root.callbacks, {})
        error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
