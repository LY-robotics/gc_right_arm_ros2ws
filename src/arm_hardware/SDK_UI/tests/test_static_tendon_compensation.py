import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from sdk_ui.arm_sdk import ArmSdk
from sdk_ui.motor_coordinates import joint_to_motor, quantize_a4
from sdk_ui.tendon_compensation import TendonModel


HORIZONTAL = (87.103289856, -2.019867214, 0.0, -0.000210459, 0.0, 0.0)
OTHER_POSE = (-10.0, -20.0, 6.0, 12.0, -20.0, 18.0)


class StaticTendonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.model = TendonModel()
        except FileNotFoundError as error:
            if os.environ.get("ARM_TENDON_LIBRARY"):
                raise
            raise unittest.SkipTest(str(error)) from error

    def test_horizontal_defaults_units_and_inactive_axes(self):
        result = self.model.evaluate(HORIZONTAL)
        self.assertAlmostEqual(result.hold_torque_nm[0], 4.291572444, places=8)
        self.assertAlmostEqual(result.compensation_deg[0], 0.127933917, places=8)
        self.assertAlmostEqual(result.compensation_deg[3], 0.021305208, places=8)
        self.assertAlmostEqual(result.rope_a_delta_mm[0], 0.069218910, places=8)
        self.assertAlmostEqual(result.rope_a_delta_n[0], 69.218910382, places=7)
        self.assertEqual(result.compensation_deg[2], 0.0)
        self.assertEqual(result.compensation_deg[4], 0.0)
        # A twist still changes the load on the compensated joints.
        straight = self.model.evaluate((35, -15, 0, 30, 0, 10))
        twisted = self.model.evaluate((35, -15, 50, 30, 20, 10))
        self.assertGreater(abs(straight.hold_torque_nm[1] - twisted.hold_torque_nm[1]), 0.1)

    def test_inverse_uses_all_encoder_coordinates(self):
        for target in (HORIZONTAL, OTHER_POSE, (0.0,) * 6):
            result = self.model.evaluate(target)
            estimated = self.model.estimate(result.effective_deg)
            for value, expected in zip(estimated, target):
                self.assertAlmostEqual(value, expected, places=7)
        with self.assertRaisesRegex(ValueError, "反解失败"):
            self.model.estimate((0, 0, 500, 0, 0, 0))

    def test_soft_ropes_need_iterative_feedback(self):
        profile = json.loads(self.model.profile.read_text(encoding="utf-8"))
        profile["rope_stiffness_n_per_mm"] = [1.0] * 6
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "soft.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            model = TendonModel(path)
            target = (27.103289856, *HORIZONTAL[1:])
            encoder = model.evaluate(target).effective_deg
            one_step = encoder[0] - model.evaluate(encoder).compensation_deg[0]
            self.assertGreater(abs(one_step - target[0]), 60)
            for value, expected in zip(model.estimate(encoder), target):
                self.assertAlmostEqual(value, expected, places=7)

    def test_profile_overrides_mass_stiffness_radius(self):
        profile = json.loads(self.model.profile.read_text(encoding="utf-8"))
        profile["mass_kg"] = [2 * mass for mass in profile["mass_kg"]]
        profile["rope_stiffness_n_per_mm"][0] *= 4
        profile["radius_mm"][0] *= 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            result = TendonModel(path).evaluate(HORIZONTAL)
            default = self.model.evaluate(HORIZONTAL)
            self.assertAlmostEqual(result.hold_torque_nm[0], 2 * default.hold_torque_nm[0])
            self.assertAlmostEqual(result.compensation_deg[0], default.compensation_deg[0] / 8)
            self.assertAlmostEqual(result.rope_a_delta_mm[0], default.rope_a_delta_mm[0] / 4)
            profile["rope_stiffness_n_per_mm"][0] = 0
            path.write_text(json.dumps(profile), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "positive"):
                TendonModel(path)

    def test_sdk_compensates_command_and_inverts_live_motor_samples(self):
        default = Path(__file__).resolve().parents[1] / "sdk_ui" / "default_hardware.json"
        config = json.loads(default.read_text(encoding="utf-8"))
        config["static_tendon_compensation"] = True
        config["motor_directions"] = [1, -1, -1, 1, 1, -1]
        config["motor_zero_offset_deg"] = [0.1, 0.2, -0.3, 0.4, 0.5, 0.6]

        def raw_angles(joint_deg):
            return [
                direction * value + offset for direction, value, offset in zip(
                    config["motor_directions"], joint_to_motor(config["joint_to_motor"], joint_deg),
                    config["motor_zero_offset_deg"]
                )
            ]

        motors = [Mock() for _ in range(6)]
        with patch("sdk_ui.arm_sdk.SocketCanBus"), patch("sdk_ui.arm_sdk.CanMotor", side_effect=motors):
            sdk = ArmSdk(config)
        for motor, raw in zip(motors, raw_angles(self.model.evaluate((0,) * 6).effective_deg)):
            motor.read_multi_turn_angle_deg.return_value = raw
        move = sdk.move_absolute(HORIZONTAL, 10.0)
        expected_raw = quantize_a4(raw_angles(self.model.evaluate(HORIZONTAL).effective_deg))
        self.assertEqual(move.target_raw_motor_deg, tuple(expected_raw))
        self.assertNotEqual(move.motor_compensation_deg[4], 0.0)  # J4 also drives M5.
        for motor, expected in zip(motors, expected_raw):
            self.assertEqual(motor.set_absolute_position_deg.call_args.args[0], expected)

        # The next read represents a different pose from the issued target.
        for motor, raw in zip(motors, raw_angles(self.model.evaluate(OTHER_POSE).effective_deg)):
            motor.read_multi_turn_angle_deg.return_value = raw
        snapshot = sdk.read_positions()
        for measured, expected in zip(snapshot.joint_deg, OTHER_POSE):
            self.assertAlmostEqual(measured, expected, places=7)
        self.assertNotEqual(snapshot.encoder_equivalent_joint_deg, snapshot.joint_deg)

        bad_target = list(HORIZONTAL)
        bad_target[0] = config["joint_position_limits_deg"][0][1] - 0.01
        for motor in motors:
            motor.reset_mock()
        with self.assertRaisesRegex(ValueError, "补偿后等效角"):
            sdk.move_absolute(bad_target, 10.0)
        for motor in motors:
            motor.set_absolute_position_deg.assert_not_called()

    def test_sdk_tendon_model_respects_reversed_j2_without_changing_motor_mapping(self):
        default = Path(__file__).resolve().parents[1] / "sdk_ui" / "default_hardware.json"
        config = json.loads(default.read_text(encoding="utf-8"))
        config["static_tendon_compensation"] = True
        config["j2_urdf_direction"] = -1
        config["joint_position_limits_deg"][1] = [-6.04, 132.36]

        target_ui = (OTHER_POSE[0], -OTHER_POSE[1], *OTHER_POSE[2:])
        native = self.model.evaluate(OTHER_POSE)
        effective_ui = (native.effective_deg[0], -native.effective_deg[1], *native.effective_deg[2:])
        raw_feedback = joint_to_motor(config["joint_to_motor"], effective_ui)
        motors = [Mock() for _ in range(6)]
        with patch("sdk_ui.arm_sdk.SocketCanBus"), patch("sdk_ui.arm_sdk.CanMotor", side_effect=motors):
            sdk = ArmSdk(config)
        for motor, raw in zip(motors, raw_feedback):
            motor.read_multi_turn_angle_deg.return_value = raw

        sdk._tendon_model.check_limits((0, -6.04, 0, 0, 0, 0), "目标位置")
        with self.assertRaisesRegex(ValueError, "J2"):
            sdk._tendon_model.check_limits((0, -6.040001, 0, 0, 0, 0), "目标位置")

        snapshot = sdk.read_positions()
        self.assertAlmostEqual(snapshot.encoder_equivalent_joint_deg[1], effective_ui[1])
        for measured, expected in zip(snapshot.joint_deg, target_ui):
            self.assertAlmostEqual(measured, expected, places=7)

        move = sdk.move_absolute(target_ui, 10.0)
        self.assertEqual(move.target_joint_deg, target_ui)
        self.assertEqual(move.target_raw_motor_deg, tuple(quantize_a4(raw_feedback)))
        self.assertAlmostEqual(move.tendon.compensation_deg[1], -native.compensation_deg[1])
        self.assertAlmostEqual(move.tendon.hold_torque_nm[1], -native.hold_torque_nm[1])
        self.assertAlmostEqual(move.tendon.rope_a_delta_mm[1], native.rope_a_delta_mm[1])
        self.assertAlmostEqual(move.predicted_joint_deg[1], target_ui[1], places=3)


if __name__ == "__main__":
    unittest.main()
