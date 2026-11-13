import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from contextlib import nullcontext

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import rospy
import replay
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_timing import scale_trajectory_speed


class ReplaySpeedTest(unittest.TestCase):
    def test_three_times_speed_preserves_path_and_recording(self):
        original = RobotTrajectory()
        original.joint_trajectory.points = [JointTrajectoryPoint(
            positions=[0.5], velocities=[0.2], accelerations=[0.1], time_from_start=rospy.Duration(9))]
        result = scale_trajectory_speed(original, 3)
        point = result.joint_trajectory.points[0]
        self.assertEqual(point.positions, [0.5])
        self.assertEqual(point.time_from_start.to_sec(), 3)
        self.assertAlmostEqual(point.velocities[0], 0.6)
        self.assertAlmostEqual(point.accelerations[0], 0.9)
        self.assertEqual(original.joint_trajectory.points[0].time_from_start.to_sec(), 9)
        self.assertEqual(scale_trajectory_speed(original), original)
        for speed in (0, -1, True, float('nan')):
            with self.assertRaises(ValueError):
                scale_trajectory_speed(original, speed)

    def test_real_control_never_reads_simulation_speed(self):
        for session, simulated in (('real', False), ('real', True), ('sim', False), ('', True)):
            with patch.dict(os.environ, {'SESSION_MODE': session}), \
                    patch.object(replay.rospy, 'get_param', return_value=simulated), \
                    patch.object(replay, 'load_config') as config:
                self.assertEqual(replay.simulation_replay_speed(), 1)
                config.assert_not_called()
        with patch.dict(os.environ, {'SESSION_MODE': 'sim'}), \
                patch.object(replay.rospy, 'get_param', return_value=True), \
                patch.object(replay, 'load_config', return_value={'simulation': {'replay_speed': 3}}):
            self.assertEqual(replay.simulation_replay_speed(), 3)

    def test_arm_and_hand_receive_simulation_speed(self):
        data = {name: RobotTrajectory() for _, name in replay.SEQUENCE}
        targets = {name: {} for group, name in replay.SEQUENCE if group == 'hand'}
        with patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(replay.arm_replay, 'execute_selected') as arm, \
                patch.object(replay.hand_replay, 'plan_and_execute_target') as hand:
            replay.execute_stages({'arm': Mock(), 'hand': Mock()}, Mock(), data, targets,
                                  saved_preparation=True, speed=3)
        for call in arm.call_args_list + hand.call_args_list:
            self.assertEqual(call.kwargs['speed'], 3)
