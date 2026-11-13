#!/usr/bin/env python3
import os
import pickle
import time

import rospy
from sensor_msgs.msg import JointState
from trajectory_store import configured_run
from task_definition import TASK_VERSION, actions
from joint_targets import target_for_group
from trajectory_timing import scale_trajectory_speed

VEL = 0.6
ACC = 0.3
SETTLE = 0.2
HAND_FILES = [action['file'] for action in actions() if action['group'] == 'hand']


def plan(group):
    result = group.plan()
    return result[1] if isinstance(result, tuple) else result


def load_pickle(name, label):
    path = configured_run(task_version=TASK_VERSION) / 'hand' / name
    with open(path, "rb") as f:
        data = pickle.load(f)
    rospy.loginfo("[hand-ui] Loaded %s %s", label, path)
    return data


def target_joint_filename(name):
    base, ext = os.path.splitext(name)
    return base + "_target_joints" + ext


def load_target_joints(name):
    return load_pickle(target_joint_filename(name), "target joints")


def load_hand_trajectory(name):
    return load_pickle(name, "hand trajectory")


def make_target_vector(group, robot, target_map):
    target = target_for_group(robot, group, target_map)
    return [target[name] for name in group.get_active_joints()]


def get_traj_goal_joint_map(traj):
    jt = traj.joint_trajectory
    if not jt.points:
        raise RuntimeError("Saved trajectory has no points")
    return dict(zip(jt.joint_names, jt.points[-1].positions))


def plan_and_execute_to_saved_goal(group, robot, saved_traj, filename):
    goal_map = get_traj_goal_joint_map(saved_traj)
    plan_and_execute_target(group, robot, goal_map, "goal of %s" % filename)


def force_trajectory_start_to_current(group, traj, label):
    names = list(traj.joint_trajectory.joint_names)
    current = fresh_hand_state(group)

    if not traj.joint_trajectory.points:
        return traj

    p0 = traj.joint_trajectory.points[0]
    p0.positions = [float(current[n]) for n in names]
    p0.velocities = [0.0] * len(names)
    p0.accelerations = [0.0] * len(names)
    p0.time_from_start = rospy.Duration.from_sec(0.0)
    rospy.loginfo("%s: synced trajectory start point to current robot state", label)
    return traj


def fresh_hand_state(group, timeout=2.0):
    names = set(group.get_active_joints())
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            message = rospy.wait_for_message('/joint_states', JointState, timeout=remaining)
        except rospy.ROSException:
            break
        state = dict(zip(message.name, message.position))
        if names.issubset(state):
            return {name: state[name] for name in names}
    raise RuntimeError('Fresh feedback for every hand joint is unavailable; task stopped')


def plan_and_execute_target(group, robot, target_map, label, speed=1.0):
    rospy.logwarn("[hand-ui] Planning to: %s", label)

    target_vec = make_target_vector(group, robot, target_map)

    group.stop()
    rospy.sleep(0.2)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_joint_value_target(target_vec)

    traj = plan(group)

    if not traj or len(traj.joint_trajectory.points) == 0:
        raise RuntimeError("[hand-ui] Planning failed: %s" % label)

    traj = force_trajectory_start_to_current(group, traj, label)

    ok = group.execute(scale_trajectory_speed(traj, speed), wait=True)
    group.stop()

    if not ok:
        raise RuntimeError("[hand-ui] Execution failed: %s" % label)

    rospy.sleep(SETTLE)
