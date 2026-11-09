"""Feedback-controlled loop and cancellation without CAN or a display."""
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_ui.demo_cycle import DemoCycle
from sdk_ui.app import ArmSdkUi
from sdk_ui.arm_sdk import PositionSnapshot
from test_live_read import FakeRoot, ImmediateThread


class DemoCycleModelTests(unittest.TestCase):
    def test_zero_demo_zero_and_two_samples(self):
        cycle = DemoCycle([10, 0, 0, 0, 0, 0], 2)
        self.assertEqual(cycle.target, (0.,) * 6)
        cycle.begin_leg(1, 100)
        self.assertFalse(cycle.observe([0] * 6, 100.1))
        self.assertFalse(cycle.observe([0.2] + [0] * 5, 100.2))
        self.assertFalse(cycle.observe([0] * 6, 100.3))
        self.assertTrue(cycle.observe([0] * 6, 100.4))
        self.assertEqual(cycle.target[0], 10)
        self.assertEqual(cycle.round_trips, 0)
        cycle.begin_leg(5, 101)
        self.assertFalse(cycle.observe([10] + [0] * 5, 102))
        self.assertTrue(cycle.observe([10] + [0] * 5, 102.3))
        self.assertEqual(cycle.target, (0.,) * 6)
        cycle.begin_leg(5, 103)
        cycle.observe([0] * 6, 104)
        self.assertTrue(cycle.observe([0] * 6, 104.3))
        self.assertEqual(cycle.round_trips, 1)

    def test_feedback_from_all_six_joints_is_required(self):
        cycle = DemoCycle([10] * 6, 2)
        cycle.begin_leg(1, 0)
        for _ in range(4):
            self.assertFalse(cycle.observe([0, 0, 0, 0, 0, 0.11], 1))

    def test_timeout_bad_feedback_and_no_overlapping_legs(self):
        cycle = DemoCycle([10] * 6, 2)
        cycle.begin_leg(1, 0)
        with self.assertRaises(RuntimeError):
            cycle.begin_leg(1, 1)
        with self.assertRaises(ValueError):
            cycle.observe([float("nan")] * 6, 1)
        with self.assertRaises(TimeoutError):
            cycle.observe([0] * 6, 11)

    def test_invalid_or_degenerate_cycle_is_rejected(self):
        for target, speed, tolerance in [([0] * 6, 1, .1), ([1] * 5, 1, .1),
                                        ([1] * 6, 0, .1), ([1] * 6, 1, 0),
                                        ([float("inf")] * 6, 1, .1)]:
            with self.subTest(target=target, speed=speed), self.assertRaises(ValueError):
                DemoCycle(target, speed, tolerance)


class DemoCycleUiTests(unittest.TestCase):
    def setUp(self):
        self.ui = ArmSdkUi.__new__(ArmSdkUi)
        self.ui.root = FakeRoot()
        self.ui.sdk = Mock()
        self.ui.busy = False
        self.ui._pending_operation = None
        self.ui._demo_cycle = None
        self.ui._cycle_key = None
        self.ui._cycle_generation = 0
        self.ui._cycle_needs_leg = False
        self.ui._cycle_started_live_read = False
        self.ui._close_after_cycle_stop = False
        self.ui.live_read_enabled = False
        self.ui._live_read_inflight = False
        self.ui._live_generation = 0
        self.ui._live_after_id = None
        self.ui.live_read_button = Mock()
        self.ui.live_read_status = Mock()
        self.ui.cycle_status = Mock()
        self.ui.cycle_tolerance_var = Mock(get=lambda: "0.1")
        self.ui.cycle_buttons = {f"demo{i}": Mock() for i in range(6)}
        self.ui.demo_vars = {f"demo{i}": [Mock(get=lambda: "10")] * 6 for i in range(6)}
        self.ui._current_speed = Mock(return_value=2.)
        self.ui._append_status = Mock()
        self.ui._show_move_result = Mock()
        self.ui._show_snapshot = Mock()
        self.ui.message_var = Mock()
        self.ui.current_labels = [Mock() for _ in range(6)]
        self.ui.motor_position_label = Mock()
        self.ui.sdk.move_absolute.return_value = Mock(duration_s=1.)
        self.thread = patch("sdk_ui.app.threading.Thread", ImmediateThread)
        self.thread.start()
        self.addCleanup(self.thread.stop)

    def finish_worker(self):
        callbacks = [key for key, (delay, _) in self.ui.root.callbacks.items() if delay == 0]
        self.assertTrue(callbacks)
        self.ui.root.run(callbacks[0])

    def start(self):
        self.ui._toggle_demo_cycle("demo1")
        self.finish_worker()

    def test_start_enables_and_moves_to_zero_and_starts_feedback(self):
        self.start()
        self.ui.sdk.motor_on.assert_called_once()
        self.ui.sdk.move_absolute.assert_called_once_with((0.,) * 6, 2.)
        self.assertTrue(self.ui.live_read_enabled)
        self.assertIsNotNone(self.ui._live_after_id)
        self.assertTrue(self.ui._demo_cycle.moving)
        self.assertEqual(self.ui.sdk._check_joint_position_limits.call_count, 2)

    def test_each_demo_has_start_and_stop_control(self):
        for i in range(6):
            with self.subTest(demo=i):
                self.ui.sdk.reset_mock()
                self.ui._toggle_demo_cycle(f"demo{i}")
                self.finish_worker()
                self.assertEqual(self.ui._cycle_key, f"demo{i}")
                self.ui._toggle_demo_cycle(f"demo{i}")
                self.finish_worker()
                self.assertIsNone(self.ui._demo_cycle)
                self.ui.sdk.emergency_stop.assert_called_once()

    def test_next_leg_waits_for_feedback_not_estimated_duration(self):
        self.start()
        snapshot = PositionSnapshot((0.,) * 6, (0.,) * 6, (0.,) * 6)
        self.ui._observe_demo_cycle(snapshot)
        self.ui._maybe_send_cycle_leg()
        self.assertEqual(self.ui.sdk.move_absolute.call_count, 1)
        self.ui._observe_demo_cycle(snapshot)
        self.ui._maybe_send_cycle_leg()
        self.assertEqual(self.ui.sdk.move_absolute.call_count, 2)
        self.assertEqual(self.ui.sdk.move_absolute.call_args.args[0], (10.,) * 6)

    def test_stop_during_inflight_operation_discards_old_success_callback(self):
        self.ui._toggle_demo_cycle("demo1")
        self.assertTrue(self.ui.busy)
        self.ui._toggle_demo_cycle("demo1")
        self.assertIsNone(self.ui._demo_cycle)
        self.finish_worker()  # Previous command's completion dispatches queued emergency.
        self.finish_worker()
        self.ui.sdk.emergency_stop.assert_called_once()
        self.assertFalse(self.ui.live_read_enabled)
        self.assertEqual(self.ui.sdk.move_absolute.call_count, 1)

    def test_manual_commands_blocked_and_emergency_cancels_cycle(self):
        self.start()
        operation = Mock()
        with patch("sdk_ui.app.messagebox.showinfo"):
            self.ui._start_worker("manual", operation)
        operation.assert_not_called()
        self.ui._start_worker("急停", self.ui.sdk.emergency_stop, priority=True)
        self.finish_worker()
        self.assertIsNone(self.ui._demo_cycle)
        self.ui.sdk.emergency_stop.assert_called_once()

    def test_read_error_stops_cycle_and_no_new_target_is_sent(self):
        self.start()
        self.ui.busy = True
        self.ui._live_read_inflight = True
        with patch("sdk_ui.app.messagebox.showerror"):
            self.ui._live_read_finished(self.ui._live_generation, None, RuntimeError("timeout"))
        self.finish_worker()
        self.assertIsNone(self.ui._demo_cycle)
        self.ui.sdk.emergency_stop.assert_called_once()
        self.assertEqual(self.ui.sdk.move_absolute.call_count, 1)

    def test_stop_during_position_poll_does_not_schedule_another_leg(self):
        self.start()
        generation = self.ui._live_generation
        self.ui.busy = True
        self.ui._live_read_inflight = True
        self.ui._toggle_demo_cycle("demo1")
        snapshot = PositionSnapshot((0.,) * 6, (0.,) * 6, (0.,) * 6)
        self.ui._live_read_finished(generation, snapshot, None)
        self.finish_worker()
        self.assertIsNone(self.ui._demo_cycle)
        self.ui.sdk.emergency_stop.assert_called_once()
        self.assertEqual(self.ui.sdk.move_absolute.call_count, 1)

    def test_timeout_stops_cycle_in_the_ui(self):
        self.start()
        deadline = self.ui._demo_cycle._deadline
        snapshot = PositionSnapshot((0.,) * 6, (0.,) * 6, (0.,) * 6)
        with patch("sdk_ui.app.time.monotonic", return_value=deadline + 1):
            self.ui._observe_demo_cycle(snapshot)
        self.finish_worker()
        self.assertIsNone(self.ui._demo_cycle)
        self.ui.sdk.emergency_stop.assert_called_once()

    def test_preexisting_live_read_remains_enabled_after_stop(self):
        self.ui.live_read_enabled = True
        self.start()
        self.ui._toggle_demo_cycle("demo1")
        self.assertTrue(self.ui.live_read_enabled)
        self.assertIsNone(self.ui._demo_cycle)

    def test_failed_target_send_cancels_loop_and_stops_motors(self):
        self.ui.sdk.move_absolute.side_effect = RuntimeError("send timeout")
        self.ui._toggle_demo_cycle("demo1")
        with patch("sdk_ui.app.messagebox.showerror"):
            self.finish_worker()
        self.finish_worker()
        self.assertIsNone(self.ui._demo_cycle)
        self.ui.sdk.emergency_stop.assert_called_once()

    def test_out_of_limit_endpoint_never_enables_motors(self):
        self.ui.sdk._check_joint_position_limits.side_effect = ValueError("outside limit")
        with patch("sdk_ui.app.messagebox.showerror"):
            self.ui._toggle_demo_cycle("demo1")
        self.ui.sdk.motor_on.assert_not_called()
        self.ui.sdk.move_absolute.assert_not_called()

    def test_close_stops_loop_before_closing_bus(self):
        self.start()
        self.ui._close()
        self.ui.sdk.close.assert_not_called()
        self.finish_worker()
        self.ui.sdk.emergency_stop.assert_called_once()
        self.ui.sdk.close.assert_called_once()
        self.assertTrue(self.ui.root.destroyed)


if __name__ == "__main__":
    unittest.main()
