from __future__ import annotations

import unittest

import numpy as np

from quick_support_demo.motion import ForwardTurnForward, TrotGait, VelocityCommand


class TrotGaitTest(unittest.TestCase):
    def setUp(self) -> None:
        self.gait = TrotGait(0.30, 0.035)

    def assert_kinematics_match(self, state) -> None:
        for leg, target in state.foot_positions_body.items():
            actual = self.gait.forward_kinematics(leg, state.joint_positions)
            np.testing.assert_allclose(actual, target, atol=1.0e-9)

    def test_zero_command_holds_symmetric_stance(self) -> None:
        state = self.gait.sample(0.37, VelocityCommand())
        self.assertTrue(all(state.stance.values()))
        self.assertAlmostEqual(state.joint_positions["FR_hip_joint"], 0.0)
        self.assertAlmostEqual(
            state.joint_positions["FR_thigh_joint"],
            state.joint_positions["RL_thigh_joint"],
        )
        self.assert_kinematics_match(state)

    def test_forward_command_uses_diagonal_trot_pairs(self) -> None:
        state = self.gait.sample(0.20, VelocityCommand(vx_mps=0.25))
        self.assertEqual(state.stance["FR"], state.stance["RL"])
        self.assertEqual(state.stance["FL"], state.stance["RR"])
        self.assertNotEqual(state.stance["FR"], state.stance["FL"])
        self.assertGreater(state.foot_positions_body["FL"][2], self.gait.nominal_foot_z)
        self.assert_kinematics_match(state)

    def test_yaw_command_changes_left_and_right_strides(self) -> None:
        state = self.gait.sample(0.05, VelocityCommand(vx_mps=0.15, wz_radps=0.5))
        fr_delta = state.foot_positions_body["FR"] - self.gait.nominal_foot("FR")
        fl_delta = state.foot_positions_body["FL"] - self.gait.nominal_foot("FL")
        self.assertNotAlmostEqual(float(fr_delta[0]), float(fl_delta[0]))
        self.assert_kinematics_match(state)


class ForwardTurnForwardTest(unittest.TestCase):
    def setUp(self) -> None:
        self.maneuver = ForwardTurnForward(
            forward_speed_mps=0.25,
            first_distance_m=0.85,
            turn_angle_rad=-np.pi / 2.0,
            turn_rate_radps=0.8,
            second_distance_m=0.90,
        )

    def test_sequence_emits_forward_right_turn_forward(self) -> None:
        first = self.maneuver.sample(0.0)
        turn = self.maneuver.sample(self.maneuver.first_duration_s + 0.1)
        second = self.maneuver.sample(
            self.maneuver.first_duration_s + self.maneuver.turn_duration_s + 0.1
        )
        complete = self.maneuver.sample(self.maneuver.total_duration_s)

        self.assertGreater(first.command.vx_mps, 0.0)
        self.assertEqual(first.command.wz_radps, 0.0)
        self.assertEqual(turn.command.vx_mps, 0.0)
        self.assertLess(turn.command.wz_radps, 0.0)
        self.assertGreater(second.command.vx_mps, 0.0)
        self.assertTrue(complete.completed)
        self.assertEqual(complete.command, VelocityCommand())

    def test_duration_matches_distances_and_turn(self) -> None:
        expected = 0.85 / 0.25 + (np.pi / 2.0) / 0.8 + 0.90 / 0.25
        self.assertAlmostEqual(self.maneuver.total_duration_s, expected)

if __name__ == "__main__":
    unittest.main()
