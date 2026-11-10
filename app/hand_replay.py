#!/usr/bin/env python3
import os
import pickle

import rospy
from trajectory_store import configured_run

VEL = 0.6
ACC = 0.3
SETTLE = 0.2
ITERATED_HAND_DELTA_SCALE = 0.4

HAND_FILES = [
    "Stage_1_Hand_thumb-only.pkl",
    "Stage_3_Move_all_fingers.pkl",
    "Stage_4_Iterated_hand_joint_angle_changes.pkl",
]

ITERATED_HAND_DELTA_FILE = HAND_FILES[2]

# Hand replay mode for non-iterated hand stages:
#   "trajectory_goal" = load each saved hand trajectory .pkl and plan to its final joint point
#   "target"          = load each *_target_joints.pkl and plan to that saved target map
HAND_REPLAY_MODE = "trajectory_goal"


def plan(group):
    result = group.plan()
    return result[1] if isinstance(result, tuple) else result


def load_pickle(name, label):
    path = configured_run() / 'hand' / name
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


def load_hand_delta(name):
    return load_pickle(name, "hand joint deltas")


def clamp(robot, joint, value):
    try:
        mn, mx = robot.get_joint(joint).bounds()
        return max(float(mn), min(float(mx), float(value)))
    except Exception:
        return float(value)


def make_target_vector(group, robot, target_map):
    active = group.get_active_joints()
    current = group.get_current_joint_values()

    vec = []
    for i, joint in enumerate(active):
        value = target_map[joint] if joint in target_map else current[i]
        vec.append(clamp(robot, joint, value))
    return vec


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
    current_names = group.get_active_joints()
    current_vals = group.get_current_joint_values()
    current = dict(zip(current_names, current_vals))

    if not traj.joint_trajectory.points:
        return traj

    p0 = traj.joint_trajectory.points[0]
    p0.positions = [float(current[n]) for n in names]
    p0.velocities = [0.0] * len(names)
    p0.accelerations = [0.0] * len(names)
    p0.time_from_start = rospy.Duration.from_sec(0.0)
    rospy.loginfo("%s: synced trajectory start point to current robot state", label)
    return traj


def plan_and_execute_target(group, robot, target_map, label):
    rospy.logwarn("[hand-ui] Planning to: %s", label)

    target_vec = make_target_vector(group, robot, target_map)

    group.stop()
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_joint_value_target(target_vec)

    traj = plan(group)

    if not traj or len(traj.joint_trajectory.points) == 0:
        raise RuntimeError("[hand-ui] Planning failed: %s" % label)

    traj = force_trajectory_start_to_current(group, traj, label)

    ok = group.execute(traj, wait=True)
    group.stop()

    if not ok:
        raise RuntimeError("[hand-ui] Execution failed: %s" % label)

    rospy.sleep(SETTLE)


def extract_delta_value(value):
    """Return a signed joint delta from one saved delta-pkl entry."""
    if isinstance(value, dict):
        if "delta" in value:
            return float(value["delta"])
        if "change" in value:
            change = value["change"]
            if isinstance(change, str):
                return float(change.replace("+", ""))
            return float(change)
        if "from" in value and "to" in value:
            return float(value["to"]) - float(value["from"])
    return float(value)


def get_current_joint_map(group):
    return dict(zip(group.get_active_joints(), group.get_current_joint_values()))


def make_target_from_current_plus_deltas(group, robot, delta_data):
    """Build target map by applying saved per-joint deltas to current joint angles."""
    current = get_current_joint_map(group)
    target = dict(current)

    for joint_name, value in delta_data.items():
        if joint_name not in current:
            rospy.logwarn("[hand-ui] Delta joint not active, skipping: %s", joint_name)
            continue
        delta = extract_delta_value(value) * ITERATED_HAND_DELTA_SCALE
        unclamped = current[joint_name] + delta
        target[joint_name] = clamp(robot, joint_name, unclamped)
        rospy.logwarn(
            "[hand-ui] %s: current=%.12f, delta=%+.12f, target=%.12f",
            joint_name, current[joint_name], delta, target[joint_name]
        )

    return target


def plan_and_execute_current_plus_deltas(group, robot, delta_data, filename):
    target_map = make_target_from_current_plus_deltas(group, robot, delta_data)
    plan_and_execute_target(
        group, robot, target_map,
        "current hand angles plus deltas from %s" % filename,
    )
