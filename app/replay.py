#!/usr/bin/env python3
"""Replay a recorded five-stage grasp task in simulation or real control."""
import argparse
import math
import os
import pickle
import sys

import moveit_commander
import rospy

import arm_replay
import hand_replay
from joint_targets import target_for_group, thumb_base_target
from sim_contacts import disable_replay_collision_checks
from task_definition import TASK_VERSION, STAGES, actions, load_task, thumb_preparation_joints
from trajectory_store import configured_run, resolve_run
from project_config import load_config

SEQUENCE = [(action['group'], action['file']) for action in actions()]


def is_simulation_replay():
    return os.environ.get('SESSION_MODE') == 'sim' and bool(rospy.get_param('/use_sim_time', False))


def simulation_replay_speed():
    # Require both the session identity and the ROS clock to identify simulation.
    if not is_simulation_replay():
        return 1.0
    return load_config()['simulation'].get('replay_speed', 3.0)


def load_recording(run, groups, robot):
    task = load_task(run)
    data, targets = {}, {}
    for action in actions():
        backend, filename = action['group'], action['file']
        with open(run / backend / filename, 'rb') as stream:
            saved = pickle.load(stream)
        trajectory = saved.joint_trajectory
        names = list(trajectory.joint_names)
        active = set(groups[backend].get_active_joints())
        if not names or len(names) != len(set(names)) or set(names) != active:
            raise ValueError('Saved joints do not match %s: %s' % (backend, filename))
        if not trajectory.points:
            raise ValueError('Empty trajectory: ' + filename)
        previous = -1.0
        for point in trajectory.points:
            if len(point.positions) != len(names) or not all(math.isfinite(v) for v in point.positions):
                raise ValueError('Invalid trajectory positions: ' + filename)
            stamp = point.time_from_start.to_sec()
            if not math.isfinite(stamp) or stamp < previous or stamp < 0:
                raise ValueError('Invalid trajectory timing: ' + filename)
            previous = stamp
        if backend == 'hand':
            with open(run / backend / action['target'], 'rb') as stream:
                target = target_for_group(robot, groups[backend], pickle.load(stream))
            tolerance = task['planning']['joint_tolerance'] + 1e-6
            if any(abs(target[name] - value) > tolerance
                   for name, value in zip(names, trajectory.points[-1].positions)):
                raise ValueError('Saved hand path does not reach its saved target: ' + filename)
            if filename == 'Stage_5_Open_hand.pkl' and any(value != 0.0 for value in target.values()):
                raise ValueError('Final hand target must be zero for every joint')
            targets[filename] = target
            if filename == 'Stage_1_Prepare_thumb_base.pkl':
                if any(abs(target[name] - task['grasp']['joint_angles'][name]) > tolerance
                       for name in thumb_preparation_joints(task)):
                    raise ValueError('Saved thumb preparation does not match its recorded grasp target')
        elif filename == 'Stage_5_Return_home.pkl':
            home = target_for_group(robot, groups['arm'], task['home_joints'])
            if any(arm_replay.angle_error(value, home[name]) > task['planning']['joint_tolerance'] + 1e-6
                   for name, value in zip(names, trajectory.points[-1].positions)):
                raise ValueError('Saved final arm path does not return to its recorded home')
        data[filename] = saved
    return task, data, targets


def execute_stages(groups, robot, data, targets, step=False, on_start=lambda: None, joint_tolerance=0.01,
                   preparation_joints=('rh_THJ5',), saved_preparation=False, speed=1.0, check_table=False):
    timing = {'speed': speed} if speed != 1.0 else {}
    for number, stage in enumerate(STAGES, 1):
        if rospy.is_shutdown():
            raise RuntimeError('ROS shut down; sequence stopped')
        label = 'Stage %d/5: %s' % (number, stage['name'])
        if step and input('Enter to execute %s (q to quit): ' % label).strip().lower() == 'q':
            return
        rospy.logwarn(label)
        for action in stage['actions']:
            if rospy.is_shutdown():
                raise RuntimeError('ROS shut down; sequence stopped')
            on_start()
            filename = action['file']
            if action['group'] == 'arm':
                trajectory = data[filename]
                trajectory.joint_trajectory.header.stamp = rospy.Time(0)
                arm_options = dict(timing)
                if check_table:
                    arm_options['check_collisions'] = True
                arm_replay.execute_selected(groups['arm'], robot, {filename: trajectory}, filename, True, **arm_options)
            else:
                target = targets[filename]
                if filename == 'Stage_1_Prepare_thumb_base.pkl' and not saved_preparation:
                    target = thumb_base_target(robot, groups['hand'],
                        {name: target[name] for name in preparation_joints}, joint_tolerance)
                hand_replay.plan_and_execute_target(groups['hand'], robot, target, filename, **timing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--run', action='store_true', help='execute all five stages')
    modes.add_argument('--step', action='store_true', help='press Enter before each stage')
    modes.add_argument('--check', action='store_true', help='validate files/groups without moving (default)')
    parser.add_argument('--trajectory-root', help='trajectory archive or specific saved run directory')
    args = parser.parse_args(rospy.myargv()[1:])
    run = (resolve_run(args.trajectory_root, task_version=TASK_VERSION) if args.trajectory_root
           else configured_run(task_version=TASK_VERSION))
    load_task(run)  # Reject old recordings before creating any motion interfaces.
    print('Using five-stage trajectory run: %s' % run, flush=True)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_hand_replay', anonymous=True)
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander('right_arm')
    hand = moveit_commander.MoveGroupCommander('right_hand')
    arm.set_max_velocity_scaling_factor(0.05)
    arm.set_max_acceleration_scaling_factor(0.05)
    hand.set_max_velocity_scaling_factor(hand_replay.VEL)
    hand.set_max_acceleration_scaling_factor(hand_replay.ACC)
    hand.set_goal_joint_tolerance(0.01)
    hand.set_goal_tolerance(0.01)
    groups = {'arm': arm, 'hand': hand}
    execution_started = False
    restore_contacts = lambda: None

    def on_start():
        nonlocal execution_started
        execution_started = True

    def stop():
        if execution_started:
            for group in groups.values():
                group.stop()

    rospy.on_shutdown(stop)
    try:
        task, data, targets = load_recording(run, groups, robot)
        print('Files, saved hand targets and right_arm/right_hand groups checked. No motion executed.', flush=True)
        print('Recorded hand lift: %.3f m' % task['planning']['lift_distance'], flush=True)
        if task['planning'].get('lift_align_gravity', False):
            print('Stage 4 replays the saved gravity-aligning rotation during the lift.', flush=True)
        for number, stage in enumerate(STAGES, 1):
            print('  Stage %d: %s' % (number, stage['name']), flush=True)
        if not (args.run or args.step):
            return
        check_table = not is_simulation_replay()
        restore_contacts = disable_replay_collision_checks(robot, protect_table=check_table)
        speed = simulation_replay_speed()
        rospy.loginfo('Replay motion speed: %.1fx', speed)
        execute_stages(groups, robot, data, targets, step=args.step, on_start=on_start,
                       joint_tolerance=task['planning']['joint_tolerance'],
                       preparation_joints=thumb_preparation_joints(task),
                       saved_preparation=task.get('hand_preparation_mode') == 'saved_target', speed=speed,
                       check_table=check_table)
    finally:
        try:
            stop()
        finally:
            try:
                restore_contacts()
            finally:
                moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
