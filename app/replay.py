#!/usr/bin/env python3
"""Run saved arm and hand stages using their original replay helpers."""
import argparse
import os
import pickle
import sys

import moveit_commander
import rospy

import arm_replay
import hand_replay
from sim_contacts import allow_grasp_contacts

ROOT = os.environ.get("TRAJECTORY_ROOT", "/project/trajectories")
SEQUENCE = [
    ("hand", "ZERO"),
    ("hand", hand_replay.HAND_FILES[0]),
    ("arm", arm_replay.ARM_FILES[0]),
    ("hand", hand_replay.HAND_FILES[1]),
    ("arm", arm_replay.ARM_FILES[1]),
    ("hand", hand_replay.ITERATED_HAND_DELTA_FILE),
    ("arm", arm_replay.ARM_FILES[2]),
    ("arm", arm_replay.ARM_FILES[3]),
    ("arm", arm_replay.ARM_FILES[4]),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--run", action="store_true", help="execute the full sequence")
    modes.add_argument("--step", action="store_true", help="press Enter before each stage")
    modes.add_argument("--check", action="store_true", help="check files and groups without moving (default)")
    parser.add_argument("--trajectory-root", default=ROOT, help="directory containing arm/ and hand/")
    args = parser.parse_args(rospy.myargv()[1:])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arm_hand_replay", anonymous=True)
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("right_arm")
    hand = moveit_commander.MoveGroupCommander("right_hand")
    arm.set_max_velocity_scaling_factor(0.05)
    arm.set_max_acceleration_scaling_factor(0.05)
    hand.set_max_velocity_scaling_factor(hand_replay.VEL)
    hand.set_max_acceleration_scaling_factor(hand_replay.ACC)
    hand.set_goal_joint_tolerance(0.01)
    hand.set_goal_tolerance(0.01)
    groups = {"arm": arm, "hand": hand}
    execution_started = False
    restore_contacts = lambda: None

    def stop():
        if not execution_started:
            return
        for group in groups.values():
            group.stop()

    rospy.on_shutdown(stop)
    try:
        data = {}
        for backend, filename in SEQUENCE:
            if filename == "ZERO":
                continue
            with open(os.path.join(args.trajectory_root, backend, filename), "rb") as stream:
                saved = pickle.load(stream)
            if filename == hand_replay.ITERATED_HAND_DELTA_FILE:
                names = list(saved)
                for value in saved.values():
                    hand_replay.extract_delta_value(value)
            else:
                trajectory = saved.joint_trajectory
                if not trajectory.points:
                    raise RuntimeError("Empty trajectory: " + filename)
                names = list(trajectory.joint_names)
                if len(names) != len(set(names)):
                    raise RuntimeError("Duplicate joint names: " + filename)
                if any(len(p.positions) != len(names) for p in trajectory.points):
                    raise RuntimeError("Invalid trajectory positions: " + filename)
            active = set(groups[backend].get_active_joints())
            if not names or not set(names).issubset(active):
                raise RuntimeError("Saved joints do not match %s: %s" % (backend, filename))
            if backend == "arm" and set(names) != active:
                raise RuntimeError("Arm trajectory must include all arm joints: " + filename)
            data[filename] = saved
        print("Files and combined right_arm/right_hand groups checked. No motion executed.", flush=True)
        for backend, filename in SEQUENCE:
            print("  %s: %s" % (backend, filename), flush=True)
        if not (args.run or args.step):
            return
        if os.environ.get("SESSION_MODE") == "sim":
            restore_contacts = allow_grasp_contacts(robot)
        for backend, filename in SEQUENCE:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shut down; sequence stopped")
            if args.step and input("Enter to execute %s: %s (q to quit): " % (backend, filename)).lower() == "q":
                return
            execution_started = True
            rospy.logwarn("Executing %s: %s", backend, filename)
            if backend == "arm":
                trajectory = data[filename]
                trajectory.joint_trajectory.header.stamp = rospy.Time(0)
                arm_replay.execute_selected(arm, robot, {filename: trajectory}, filename, True)
            elif filename == "ZERO":
                hand_replay.plan_and_execute_target(
                    hand, robot, {j: 0.0 for j in hand.get_active_joints()}, "all hand joints zero",
                )
            elif filename == hand_replay.ITERATED_HAND_DELTA_FILE:
                hand_replay.plan_and_execute_current_plus_deltas(hand, robot, data[filename], filename)
            else:
                hand_replay.plan_and_execute_to_saved_goal(hand, robot, data[filename], filename)
    finally:
        stop()
        restore_contacts()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
