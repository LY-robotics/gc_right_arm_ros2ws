import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from sdk_ui.app import ArmSdkUi, DEFAULT_HARDWARE_CONFIG
from test_live_read import FakeRoot, ImmediateThread


class CartesianUiTest(unittest.TestCase):
    def setUp(self):
        ui = ArmSdkUi.__new__(ArmSdkUi)
        self.ui = ui
        ui.root = FakeRoot()
        ui.sdk = Mock()
        ui.sdk.config = {"j2_urdf_direction": -1}
        ui.busy = False
        ui._pending_operation = None
        ui._cartesian_cancel = None
        ui._close_after_cartesian_stop = False
        ui.message_var = Mock()
        ui._append_status = Mock()
        ui.cartesian_xyz_vars = [Mock(get=Mock(return_value=value)) for value in ("0.1", "-0.2", "0.3")]
        ui.cartesian_speed_var = Mock(get=Mock(return_value="12"))
        ui.cartesian_directory_var = Mock(get=Mock(return_value=str(Path.cwd())))
        ui._current_speed = Mock(return_value=8.0)
        ui.cartesian_start_button = Mock()
        ui.cartesian_stop_button = Mock()
        ui.cartesian_pose_var = Mock()
        ui.cartesian_status_var = Mock()

    def test_xyz_speed_units_and_live_feedback_are_connected_to_executor(self):
        def execute(sdk, model, target, speed, joint_speed, trace_path, *, cancel_event, on_sample):
            self.assertIs(sdk, self.ui.sdk)
            self.assertEqual(target, (0.1, -0.2, 0.3))
            self.assertEqual(speed, 0.012)
            self.assertEqual(joint_speed, 8.0)
            self.assertEqual(trace_path.suffix, ".csv")
            self.assertFalse(cancel_event.is_set())
            on_sample(SimpleNamespace(
                waypoint_index=1, position_m=target, rpy_deg=(0.0, 1.0, 2.0),
                speed_m_s=0.012, cross_track_error_m=0.0003,
            ))
            return SimpleNamespace(status="completed", samples=1, trace_path=trace_path)

        with patch("sdk_ui.app.threading.Thread", ImmediateThread), patch(
            "sdk_ui.kinematics.ArmKinematics"
        ) as kinematics, patch("sdk_ui.cartesian_motion.execute_cartesian_line", side_effect=execute):
            self.ui._start_cartesian()
            callback = next(key for key, (delay, _) in self.ui.root.callbacks.items() if delay == 0)
            self.ui.root.run(callback)

        kinematics.assert_called_once_with(j2_urdf_direction=-1)
        self.assertIsNone(self.ui._cartesian_cancel)
        self.assertFalse(self.ui.busy)
        self.assertIn("12.00", self.ui.cartesian_pose_var.set.call_args.args[0])
        self.assertIn("已到达目标", self.ui.cartesian_status_var.set.call_args.args[0])
        self.assertIn(".csv", self.ui.cartesian_status_var.set.call_args.args[0])

    def test_fk_uses_the_applied_j2_direction(self):
        self.ui.sdk.read_positions.return_value = SimpleNamespace(joint_deg=(0, 30, 0, 0, 0, 0))
        with patch("sdk_ui.app.threading.Thread", ImmediateThread), patch(
            "sdk_ui.kinematics.ArmKinematics"
        ) as kinematics:
            kinematics.return_value.forward.return_value = SimpleNamespace(
                position_m=(0.1, 0.2, 0.3), rpy_deg=(0.0, 0.0, 0.0))
            self.ui._read_cartesian_pose()
            callback = next(key for key, (delay, _) in self.ui.root.callbacks.items() if delay == 0)
            self.ui.root.run(callback)

        kinematics.assert_called_once_with(j2_urdf_direction=-1)
        kinematics.return_value.forward.assert_called_once_with((0, 30, 0, 0, 0, 0))
        self.ui.cartesian_xyz_vars[0].set.assert_called_once_with("0.1000000")

    def test_apply_config_carries_j2_checkbox_and_ui_limits(self):
        config = json.loads(DEFAULT_HARDWARE_CONFIG.read_text(encoding="utf-8"))
        config["joint_position_limits_deg"][1] = [-6.04, 132.36]
        self.ui.hardware_config = config
        self.ui.can_var = Mock(get=Mock(return_value=config["can_interface"]))
        self.ui.tendon_enabled_var = Mock(get=Mock(return_value=False))
        self.ui.tendon_profile_var = Mock(get=Mock(return_value=""))
        self.ui.j2_reversed_var = Mock(get=Mock(return_value=True))
        self.ui.motor_id_vars = [Mock(get=Mock(return_value=str(i))) for i in config["motor_ids"]]

        applied = self.ui._config_from_widgets()
        self.assertEqual(applied["j2_urdf_direction"], -1)
        self.assertEqual(applied["joint_position_limits_deg"][1], [-6.04, 132.36])

    def test_priority_hardware_stop_signals_active_trajectory_before_queuing(self):
        self.ui.busy = True
        self.ui._live_read_inflight = False
        cancel = threading.Event()
        self.ui._cartesian_cancel = cancel
        stop = Mock()
        self.ui._start_worker("急停", stop, priority=True)
        self.assertTrue(cancel.is_set())
        self.assertEqual(self.ui._pending_operation, ("急停", stop, None, True))
        stop.assert_not_called()

    def test_close_cancels_running_motion_without_destroying_window(self):
        self.ui._stop_live_read = Mock()
        self.ui._cartesian_cancel = threading.Event()
        self.ui._close()
        self.assertTrue(self.ui._cartesian_cancel.is_set())
        self.assertTrue(self.ui._close_after_cartesian_stop)
        self.assertFalse(self.ui.root.destroyed)


if __name__ == "__main__":
    unittest.main()
