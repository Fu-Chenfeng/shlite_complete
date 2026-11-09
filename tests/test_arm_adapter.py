"""Run with ROS Noetic Python; no ROS master or hardware is needed."""
import copy
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
import ur5_adapter as adapter
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy


def make_goal():
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["ra_" + name for name in reversed(adapter.JOINTS)]
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=[0.1 * i for i in range(6)], time_from_start=rospy.Duration(3),
    )]
    return goal


class AdapterTest(unittest.TestCase):
    def test_mapping_preserves_joint_order_positions_and_timing(self):
        goal = make_goal()
        original = copy.deepcopy(goal)
        result = adapter.driver_goal(goal)
        self.assertEqual(result.trajectory.joint_names, list(reversed(adapter.JOINTS)))
        self.assertEqual(result.trajectory.points, goal.trajectory.points)
        self.assertEqual(goal, original)

    def test_partial_duplicate_and_foreign_joints_are_rejected(self):
        for names in (
            list(adapter.JOINTS)[:-1],
            [adapter.JOINTS[0]] * 6,
            ["rh_FFJ1"] + list(adapter.JOINTS)[1:],
        ):
            with self.subTest(names=names), self.assertRaises(ValueError):
                adapter.map_names(names)

    def relay(self):
        instance = object.__new__(adapter.Adapter)
        instance.lock = threading.RLock()
        instance.server = Mock()
        instance.client = Mock()
        instance.server.is_preempt_requested.return_value = False
        instance.client.wait_for_server.return_value = True
        return instance

    def test_invalid_goal_never_reaches_controller(self):
        relay = self.relay()
        goal = make_goal()
        goal.trajectory.points[0].positions = [0.0]
        relay.execute(goal)
        relay.server.set_aborted.assert_called_once()
        relay.client.send_goal.assert_not_called()

    def test_controller_failure_propagates_to_caller(self):
        relay = self.relay()
        relay.client.wait_for_result.return_value = True
        relay.client.get_state.return_value = adapter.GoalStatus.ABORTED
        relay.client.get_result.return_value = FollowJointTrajectoryResult(error_code=-5)
        relay.execute(make_goal())
        relay.server.set_aborted.assert_called_once()
        relay.server.set_succeeded.assert_not_called()

    def test_preemption_cancels_driver_goal(self):
        relay = self.relay()
        relay.client.wait_for_result.side_effect = [False, True]
        relay.server.is_preempt_requested.side_effect = [False, True]
        relay.execute(make_goal())
        relay.client.cancel_goal.assert_called_once()
        relay.server.set_preempted.assert_called_once()
        relay.server.set_succeeded.assert_not_called()

    def test_cancel_timeout_reports_failure(self):
        relay = self.relay()
        relay.client.wait_for_result.side_effect = [False, False]
        relay.server.is_preempt_requested.side_effect = [False, True]
        relay.execute(make_goal())
        relay.client.cancel_goal.assert_called_once()
        relay.server.set_aborted.assert_called_once()
        relay.server.set_succeeded.assert_not_called()


if __name__ == "__main__":
    unittest.main()
