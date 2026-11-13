"""Fresh-feedback checks for planning and replaying hand commands."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import hand_replay
import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class HandFeedbackTest(unittest.TestCase):
    def test_fresh_state_skips_arm_only_messages(self):
        group = Mock()
        group.get_active_joints.return_value = ['rh_FFJ1', 'rh_THJ1']
        messages = [JointState(name=['ra_elbow_joint'], position=[0.5]),
                    JointState(name=['rh_THJ1', 'rh_FFJ1'], position=[0.2, 0.1])]
        with patch.object(hand_replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(hand_replay.rospy, 'wait_for_message', side_effect=messages) as receive:
            state = hand_replay.fresh_hand_state(group)
        self.assertEqual(state, {'rh_FFJ1': 0.1, 'rh_THJ1': 0.2})
        self.assertEqual(receive.call_count, 2)

    def test_absent_feedback_stops_without_motion(self):
        group = Mock()
        group.get_active_joints.return_value = ['rh_FFJ1']
        with patch.object(hand_replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(hand_replay.rospy, 'wait_for_message', side_effect=rospy.ROSException('offline')):
            with self.assertRaisesRegex(RuntimeError, 'Fresh feedback'):
                hand_replay.fresh_hand_state(group)
        group.execute.assert_not_called()

    def test_trajectory_start_uses_fresh_feedback_and_preserves_final_target(self):
        group = Mock()
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['rh_THJ1', 'rh_FFJ1']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.9, 0.9]),
                                              JointTrajectoryPoint(positions=[0.0, 0.0])]
        with patch.object(hand_replay, 'fresh_hand_state', return_value={'rh_FFJ1': 0.1, 'rh_THJ1': 0.2}):
            hand_replay.force_trajectory_start_to_current(group, trajectory, 'reset')
        self.assertEqual(trajectory.joint_trajectory.points[0].positions, [0.2, 0.1])
        self.assertEqual(trajectory.joint_trajectory.points[-1].positions, [0.0, 0.0])
        group.get_current_joint_values.assert_not_called()



if __name__ == '__main__':
    unittest.main()
