#!/usr/bin/env python3
"""Arm replay helpers preserving the working UR5 start checks and timing."""
import os
import pickle
import math

import rospy
from trajectory_store import configured_run

START_TOL_ARM = 0.02
SETTLE_AFTER_START_MOVE = 0.5
SETTLE_AFTER_REPLAY = 0.2

ARM_FILES = [
    "Stage_2_Move_arm_to_captured_hand_pose.pkl",
    "Stage_4_Move_arm_with_palm_shift.pkl",
    "Stage_5a_Move_arm_to_additional_offset_pose.pkl",
    "Stage_5_Straight-line_Z_lift.pkl",
    "After_Stage_5_Planned_joint_move_to_home.pkl",
]


def nice_label(filename):
    return os.path.splitext(filename)[0].replace("_", " ")


def normalize_ur5_joint_names(traj):
    jt = traj.joint_trajectory
    jt.joint_names = [
        name.replace("ra_", "", 1) if name.startswith("ra_") else name
        for name in jt.joint_names
    ]
    return traj


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def nearest_equivalent_angle(current, target):
    return current + math.atan2(math.sin(target - current), math.cos(target - current))


def plan_traj(group):
    result = group.plan()
    return result[1] if isinstance(result, tuple) else result


def load_traj(filename):
    path = configured_run() / 'arm' / filename
    with open(path, "rb") as f:
        traj = pickle.load(f)
    return normalize_ur5_joint_names(traj)


def get_traj_start_joint_map(traj):
    jt = traj.joint_trajectory
    if not jt.points:
        raise RuntimeError("Trajectory has no points")
    return dict(zip(jt.joint_names, jt.points[0].positions))


def get_current_joint_map(group):
    return dict(zip(group.get_active_joints(), group.get_current_joint_values()))


def max_start_error(group, traj):
    current = get_current_joint_map(group)
    target = get_traj_start_joint_map(traj)
    errors = {
        joint: angle_error(current[joint], value)
        for joint, value in target.items()
        if joint in current
    }
    if not errors:
        raise RuntimeError(
            "No matching joints. MoveIt joints=%s, trajectory joints=%s"
            % (list(current.keys()), list(target.keys()))
        )
    max_joint = max(errors, key=errors.get)
    return errors[max_joint], max_joint, errors


def clamp_joint(robot, joint_name, value):
    try:
        mn, mx = robot.get_joint(joint_name).bounds()
        return max(float(mn), min(float(mx), float(value)))
    except Exception:
        return float(value)


def make_start_target_vec(group, traj, robot):
    target = get_traj_start_joint_map(traj)
    active = group.get_active_joints()
    current = group.get_current_joint_values()
    vec = []
    for i, joint_name in enumerate(active):
        value = target[joint_name] if joint_name in target else current[i]
        value = nearest_equivalent_angle(current[i], value)
        vec.append(clamp_joint(robot, joint_name, value))
    return vec


def plan_and_execute_to_start(group, traj, robot, filename):
    target_vec = make_start_target_vec(group, traj, robot)

    group.stop()
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_joint_value_target(target_vec)

    pre_traj = plan_traj(group)
    if not pre_traj or len(pre_traj.joint_trajectory.points) == 0:
        raise RuntimeError("Could not plan to trajectory start: %s" % filename)

    ok = group.execute(pre_traj, wait=True)
    group.stop()
    if not ok:
        raise RuntimeError("Failed to execute motion to trajectory start: %s" % filename)

    rospy.sleep(SETTLE_AFTER_START_MOVE)

    max_err, joint, errors = max_start_error(group, traj)
    if max_err > START_TOL_ARM:
        raise RuntimeError(
            "Robot did not reach trajectory start. Worst joint=%s error=%.6f tolerance=%.6f errors=%s"
            % (joint, max_err, START_TOL_ARM, errors)
        )


def execute_selected(arm, robot, trajectories, filename, move_to_start):
    traj = trajectories[filename]

    max_err, joint, _ = max_start_error(arm, traj)
    lines = ["Selected: %s" % filename]
    lines.append("Start check: worst joint=%s error=%.6f" % (joint, max_err))

    if max_err > START_TOL_ARM:
        if move_to_start:
            lines.append("Moving to trajectory start first...")
            plan_and_execute_to_start(arm, traj, robot, filename)
        else:
            raise RuntimeError(
                "Current robot state is not at trajectory start. Worst joint=%s error=%.6f tolerance=%.6f"
                % (joint, max_err, START_TOL_ARM)
            )

    lines.append("Executing trajectory...")
    ok = arm.execute(traj, wait=True)
    arm.stop()
    if not ok:
        raise RuntimeError("MoveIt failed to execute trajectory: %s" % filename)

    rospy.sleep(SETTLE_AFTER_REPLAY)
    lines.append("Done: %s" % filename)
    return lines
