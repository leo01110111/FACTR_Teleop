import sys
import time
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_DIR / "scripts" / "isaac_cumotion"
sys.path.insert(0, str(SCRIPTS_DIR))

from isaac6_cumotion_stream_server import (  # noqa: E402
    DEFAULT_CONFIG_DIR,
    REQUEST_SCHEMA,
    PassThroughPolicy,
    _compute_response,
    _load_collision_spheres,
    _opposite_side,
    _parse_sides,
    _quat_wxyz_rotation,
    _yaw_rotation,
)


class TestIsaacCuMotionStreamServer(unittest.TestCase):
    def _request(self, *, stamp=None, desired=None):
        desired = desired if desired is not None else [0.01, -0.02, 0.03, 0.0, 0.0, 0.01]
        return {
            "schema": REQUEST_SCHEMA,
            "sequence": 42,
            "stamp": time.time() if stamp is None else stamp,
            "active_sides": ["right"],
            "arms": {
                "right": {
                    "q_current": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "q_desired": desired,
                }
            },
            "limits": {"max_joint_step_rad": 0.05},
        }

    def test_parse_sides(self):
        self.assertEqual(_parse_sides("right,left,right"), ("right", "left"))
        with self.assertRaises(ValueError):
            _parse_sides("center")

    def test_opposite_side(self):
        self.assertEqual(_opposite_side("right"), "left")
        self.assertEqual(_opposite_side("left"), "right")

    def test_scene_quaternion_matches_yaw_rotation(self):
        quat_rotation = _quat_wxyz_rotation([0.70710678, 0.0, 0.0, -0.70710678])
        yaw_rotation = _yaw_rotation(-1.57079632679)
        self.assertTrue((abs(quat_rotation - yaw_rotation) < 1e-6).all())

    def test_pass_through_response(self):
        response = _compute_response(
            PassThroughPolicy(),
            self._request(),
            controller_dt=0.002,
            stale_after_s=0.10,
            require_other_arm_state=False,
            policy_name="pass_through",
        )
        self.assertTrue(response["ok"])
        self.assertEqual(response["sequence"], 42)
        self.assertEqual(response["policy"], "pass_through")
        self.assertEqual(response["mode"], "pass_through")
        self.assertEqual(response["arms"]["right"]["q_safe"], [0.01, -0.02, 0.03, 0.0, 0.0, 0.01])

    def test_pass_through_step_clips_large_target(self):
        response = _compute_response(
            PassThroughPolicy(),
            self._request(desired=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
            controller_dt=0.002,
            stale_after_s=0.10,
            require_other_arm_state=False,
            policy_name="pass_through",
        )
        self.assertTrue(response["ok"])
        self.assertEqual(response["policy"], "pass_through")
        self.assertEqual(response["mode"], "filtered")
        self.assertEqual(response["reason"], "step_clipped")
        self.assertEqual(response["arms"]["right"]["q_safe"], [0.05, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_response_policy_can_be_explicitly_rmp(self):
        response = _compute_response(
            PassThroughPolicy(),
            self._request(),
            controller_dt=0.002,
            stale_after_s=0.10,
            require_other_arm_state=False,
            policy_name="rmp",
        )
        self.assertTrue(response["ok"])
        self.assertEqual(response["policy"], "rmp")

    def test_stale_request_fails_closed_with_policy(self):
        response = _compute_response(
            PassThroughPolicy(),
            self._request(stamp=time.time() - 1.0),
            controller_dt=0.002,
            stale_after_s=0.10,
            require_other_arm_state=False,
            policy_name="rmp",
        )
        self.assertFalse(response["ok"])
        self.assertEqual(response["policy"], "rmp")
        self.assertEqual(response["mode"], "hold")
        self.assertIn("input age", response["reason"])

    def test_collision_spheres_include_robotiq_gripper(self):
        spheres = _load_collision_spheres(DEFAULT_CONFIG_DIR / "robot.xrdf")
        links = {link_name for link_name, _, _ in spheres}
        self.assertIn("wrist_3_link", links)
        self.assertIn("tool0", links)
        self.assertIn("pinch_center", links)
        self.assertIn("robotiq_base", links)
        self.assertIn("robotiq_right_finger", links)
        self.assertIn("robotiq_left_finger", links)
        self.assertIn("robotiq_right_pad", links)
        self.assertIn("robotiq_left_pad", links)


if __name__ == "__main__":
    unittest.main()
