#!/usr/bin/env python3
"""Map the combined ra_ joints onto ur5_docker's original UR controller."""
import copy
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryResult,
    JointTrajectoryControllerState,
)
from sensor_msgs.msg import JointState

JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


def map_names(names, to_driver=False):
    expected = {"ra_" + name for name in JOINTS} if to_driver else set(JOINTS)
    if len(names) != len(JOINTS) or set(names) != expected:
        raise ValueError("Expected all six UR5 joints exactly once, got %s" % list(names))
    return [name[3:] if to_driver else "ra_" + name for name in names]


def driver_goal(goal):
    result = copy.deepcopy(goal)
    result.trajectory.joint_names = map_names(goal.trajectory.joint_names, to_driver=True)
    if not result.trajectory.points:
        raise ValueError("Trajectory has no points")
    for point in result.trajectory.points:
        if len(point.positions) != 6:
            raise ValueError("Each trajectory point must contain six positions")
        for values in (point.velocities, point.accelerations, point.effort):
            if values and len(values) != 6:
                raise ValueError("Trajectory point arrays must contain six values or be empty")
    for tolerance in result.path_tolerance + result.goal_tolerance:
        if tolerance.name not in {"ra_" + name for name in JOINTS}:
            raise ValueError("Unknown tolerance joint: %s" % tolerance.name)
        tolerance.name = tolerance.name[3:]
    return result


class Adapter:
    def __init__(self):
        self.lock = threading.RLock()
        self.client = actionlib.SimpleActionClient(
            "/ur5_driver/scaled_pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.server = actionlib.SimpleActionServer(
            "/ra_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction, execute_cb=self.execute, auto_start=False,
        )
        self.server.register_preempt_callback(self.cancel)
        self.joints = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.state = rospy.Publisher(
            "/ra_trajectory_controller/state", JointTrajectoryControllerState, queue_size=10,
        )
        rospy.Subscriber("/ur5_driver/joint_states", JointState, self.joint_state, queue_size=10)
        rospy.Subscriber(
            "/ur5_driver/scaled_pos_joint_traj_controller/state",
            JointTrajectoryControllerState, self.controller_state, queue_size=10,
        )
        rospy.on_shutdown(self.cancel)
        self.server.start()

    def joint_state(self, message):
        result = copy.deepcopy(message)
        try:
            result.name = map_names(message.name)
        except ValueError as exc:
            rospy.logwarn_throttle(5, str(exc))
            return
        self.joints.publish(result)

    def controller_state(self, message):
        result = copy.deepcopy(message)
        try:
            result.joint_names = map_names(message.joint_names)
        except ValueError as exc:
            rospy.logwarn_throttle(5, str(exc))
            return
        self.state.publish(result)

    def feedback(self, message):
        result = copy.deepcopy(message)
        result.joint_names = map_names(message.joint_names)
        self.server.publish_feedback(result)

    def cancel(self):
        with self.lock:
            self.client.cancel_goal()

    def execute(self, goal):
        try:
            mapped = driver_goal(goal)
        except ValueError as exc:
            self.server.set_aborted(FollowJointTrajectoryResult(
                error_code=FollowJointTrajectoryResult.INVALID_JOINTS,
                error_string=str(exc),
            ))
            return
        if not self.client.wait_for_server(rospy.Duration(5)):
            self.server.set_aborted(FollowJointTrajectoryResult(
                error_code=FollowJointTrajectoryResult.INVALID_GOAL,
                error_string="UR5 scaled trajectory controller is unavailable",
            ))
            return
        with self.lock:
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                self.server.set_preempted()
                return
            self.client.send_goal(mapped, feedback_cb=self.feedback)
        while not self.client.wait_for_result(rospy.Duration(0.1)):
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                self.cancel()
                # Do not let the coordinator advance while cancellation is pending.
                if not self.client.wait_for_result(rospy.Duration(5)):
                    self.server.set_aborted(FollowJointTrajectoryResult(
                        error_code=FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED,
                        error_string="UR5 did not acknowledge cancellation",
                    ))
                else:
                    self.server.set_preempted(self.client.get_result())
                return
        result = self.client.get_result()
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED and result and result.error_code == 0:
            self.server.set_succeeded(result)
        elif state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            self.server.set_preempted(result)
        else:
            self.server.set_aborted(result, "UR5 controller did not succeed")


if __name__ == "__main__":
    rospy.init_node("ur5_combined_adapter")
    adapter = Adapter()
    rospy.spin()
