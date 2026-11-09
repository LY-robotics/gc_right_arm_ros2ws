import math
import unittest

import numpy as np

from sdk_ui.kinematics import ArmKinematics, KinematicsError


class CartesianKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ArmKinematics()

    def test_fk_in_base_and_joint_one_rotation(self):
        self.assertEqual(self.model.base_frame, "base")
        self.assertEqual(self.model.tip_frame, "Arm6")
        np.testing.assert_allclose(self.model.velocity_limits_dps, (10,) * 6, atol=1e-4)
        pose = self.model.forward((0, 0, 0, 0, 0, 0))
        np.testing.assert_allclose(
            pose.position_m, (-0.0278791235, 0.5509521071, 0.0744441476), atol=1e-9
        )
        x, y, z = pose.position_m
        np.testing.assert_allclose(
            self.model.forward((90, 0, 0, 0, 0, 0)).position_m, (-y, x, z), atol=1e-9
        )
        self.assertAlmostEqual(np.linalg.norm(pose.quaternion_xyzw), 1.0)

    def test_inverse_matches_pose_from_nearby_seed(self):
        joints = (20, -35, 20, 55, -15, 10)
        pose = self.model.forward(joints)
        # The two quaternion signs describe the same orientation.
        solved = self.model.inverse(
            pose.position_m, tuple(-value for value in pose.quaternion_xyzw),
            (25, -40, 24, 50, -10, 14),
        )
        np.testing.assert_allclose(solved, joints, atol=0.01)

    def test_reversed_j2_preserves_fk_and_returns_sdk_angles_from_ik(self):
        model = ArmKinematics(j2_urdf_direction=-1)
        joints = (20, 35, 20, 55, -15, 10)
        pose = self.model.forward((20, -35, 20, 55, -15, 10))
        self.assertEqual(model.forward(joints), pose)
        solved = model.inverse(pose.position_m, pose.quaternion_xyzw, (25, 40, 24, 50, -10, 14))
        np.testing.assert_allclose(solved, joints, atol=0.01)
        np.testing.assert_allclose(model.position_limits_deg[1], (-6.04, 132.36), atol=1e-8)

    def test_reversed_j2_plans_positive_sdk_angles_with_matching_limits(self):
        model = ArmKinematics(j2_urdf_direction=-1)
        start = (20, 35, 20, 55, -15, 10)
        target = np.array(model.forward(start).position_m) + (-0.008, 0.004, 0.010)
        sdk_limits = list(self.model.position_limits_deg)
        sdk_limits[1] = (-6.04, 132.36)
        line = model.plan_line(start, target, joint_limits_deg=sdk_limits)
        self.assertTrue(line)
        self.assertTrue(all(6.04 < point.joint_deg[1] <= 132.36 for point in line))
        np.testing.assert_allclose(
            model.forward(line[-1].joint_deg).position_m, target, atol=0.00005
        )
        sdk_limits[1] = (-6.04, 30)
        with self.assertRaisesRegex(KinematicsError, r"J2 反馈 35\.0+°.*30\.0+°"):
            model.plan_line(start, target, joint_limits_deg=sdk_limits)

    def test_line_waypoints_keep_orientation_and_stay_on_segment(self):
        start_joints = (20, -35, 20, 55, -15, 10)
        start = self.model.forward(start_joints)
        delta = np.array((-0.008, 0.004, 0.010))
        target = np.array(start.position_m) + delta
        line = self.model.plan_line(start_joints, target)
        self.assertGreater(len(line), 1)
        np.testing.assert_allclose(line[-1].position_m, target, atol=1e-12)
        self.assertAlmostEqual(sum(point.distance_m for point in line), np.linalg.norm(delta))
        distance = 0.0
        previous = np.array(start_joints)
        for point in line:
            self.assertGreater(point.distance_m, 0)
            self.assertLessEqual(point.distance_m, 0.005)
            self.assertLessEqual(np.max(np.abs(np.array(point.joint_deg) - previous)), 5.0)
            distance += point.distance_m
            reference = np.array(start.position_m) + delta * (distance / np.linalg.norm(delta))
            np.testing.assert_allclose(point.position_m, reference, atol=1e-12)
            pose = self.model.forward(point.joint_deg)
            self.assertLessEqual(np.linalg.norm(np.array(pose.position_m) - reference), 0.00005)
            cosine = abs(np.dot(pose.quaternion_xyzw, start.quaternion_xyzw))
            self.assertLessEqual(2 * math.degrees(math.acos(min(1.0, cosine))), 0.02)
            previous = np.array(point.joint_deg)

    def test_straight_reference_subdivides_near_full_extension(self):
        joints = (87.103, -2.019867, 0, -0.000210, 0, 0)
        start = self.model.forward(joints)
        target = np.array(start.position_m) + (0.001, 0, 0)
        line = self.model.plan_line(joints, target)
        self.assertGreater(len(line), 1)
        np.testing.assert_allclose(
            self.model.forward(line[-1].joint_deg).position_m, target, atol=0.00005
        )

    def test_unreachable_limit_and_cancelled_paths_raise(self):
        joints = (20, -35, 20, 55, -15, 10)
        start = self.model.forward(joints)
        with self.assertRaisesRegex(KinematicsError, "最大伸展范围"):
            self.model.plan_line(joints, (1, 0, 0))
        with self.assertRaisesRegex(KinematicsError, "关节限位"):
            self.model.plan_line((200, *joints[1:]), start.position_m)
        with self.assertRaisesRegex(KinematicsError, "已取消"):
            self.model.plan_line(joints, start.position_m, cancelled=lambda: True)
        target = np.array(start.position_m) + (0, 0, -0.01)
        narrow_limits = tuple((angle - 0.002, angle + 0.002) for angle in joints)
        with self.assertRaisesRegex(KinematicsError, "无法连续规划"):
            self.model.plan_line(joints, target, joint_limits_deg=narrow_limits)
        self.assertEqual(self.model.plan_line(joints, start.position_m), ())


if __name__ == "__main__":
    unittest.main()
