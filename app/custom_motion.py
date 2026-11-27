#!/usr/bin/env python3
"""Capture RViz goals; plan, execute, save and replay custom joint sequences."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import pickle
import sys

import moveit_commander
import rospy
from moveit_msgs.msg import RobotState

from custom_trajectory import TASK_VERSION, RVIZ_GOAL_TOPIC, load_task, stages, validate_sequence
from joint_targets import target_for_group
from project_config import load_config
from trajectory_store import archive_root, create_run, finish_run


def require_simulation():
    if os.environ.get('SESSION_MODE') != 'sim' or not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Custom trajectory creation is available only in simulation')


def capture(robot, groups, source):
    require_simulation()
    if source == 'rviz':
        try:
            state = rospy.wait_for_message(RVIZ_GOAL_TOPIC, RobotState, timeout=5)
        except rospy.ROSException as exc:
            raise RuntimeError('No RViz goal received. Restart simulation to load the custom trajectory RViz bridge, '
                               'and enable the MotionPlanning display.') from exc
    else:
        state = robot.get_current_state()
    js = state.joint_state
    if len(js.name) != len(js.position) or len(set(js.name)) != len(js.name):
        raise ValueError('Robot state has incomplete or duplicate joints')
    values = dict(zip(js.name, js.position))
    result = {'source': source, 'bounds': {}}
    for backend, group in groups.items():
        names = group.get_active_joints()
        if not set(names).issubset(values):
            raise ValueError('Robot state is missing %s joints' % backend)
        requested = {name: values[name] for name in names}
        # RViz's unedited joints also inherit Gazebo feedback, which can drift
        # just beyond a mechanical stop. Normalize this small simulation noise.
        for name, value in requested.items():
            lo, hi = robot.get_joint(name).bounds()
            if math.isfinite(value) and lo - .01 <= value <= hi + .01:
                requested[name] = min(hi, max(lo, value))
            result['bounds'][name] = [lo, hi]
        result[backend] = target_for_group(robot, group, requested)
    return result


def endpoint(state, trajectory):
    result = copy.deepcopy(state)
    jt = trajectory.joint_trajectory
    values = dict(zip(jt.joint_names, jt.points[-1].positions))
    js = result.joint_state
    js.position = [values.get(n, v) for n, v in zip(js.name, js.position)]
    return result


def configure_groups(groups, options):
    for backend, group in groups.items():
        group.set_planning_time(options['planning_time'])
        group.set_num_planning_attempts(options['planning_attempts'])
        group.set_max_velocity_scaling_factor(options[backend + '_velocity'])
        group.set_max_acceleration_scaling_factor(options[backend + '_acceleration'])
        group.set_goal_joint_tolerance(options['joint_tolerance'])


def plan_sequence(robot, groups, sequence):
    validate_sequence(sequence)
    # Validate every angle before any planning or execution.
    for task in sequence['tasks']:
        for backend, group in groups.items():
            target_for_group(robot, group, task[backend])
    state = robot.get_current_state()
    planned = []
    for task, stage in zip(sequence['tasks'], stages(sequence)):
        for action in stage['actions']:
            if rospy.is_shutdown():
                raise RuntimeError('Custom planning interrupted')
            backend = action['group']
            group = groups[backend]
            group.clear_pose_targets()
            group.set_start_state(state)
            group.set_joint_value_target(task[backend])
            rospy.loginfo('Planning %s: %s', stage['name'], backend)
            result = group.plan()
            if isinstance(result, tuple) and not result[0]:
                raise RuntimeError('Planning failed for %s (%s); no motion started' % (task['name'], backend))
            trajectory = result[1] if isinstance(result, tuple) else result
            if not trajectory or not trajectory.joint_trajectory.points:
                raise RuntimeError('Empty plan for %s (%s); no motion started' % (task['name'], backend))
            planned.append((action, trajectory, task[backend]))
            state = endpoint(state, trajectory)
    return planned


def execute_plans(groups, planned, tolerance):
    for action, trajectory, target in planned:
        if rospy.is_shutdown():
            raise RuntimeError('Custom execution interrupted')
        group = groups[action['group']]
        current = dict(zip(group.get_active_joints(), group.get_current_joint_values()))
        jt = trajectory.joint_trajectory
        error = max(abs(current[n] - v) for n, v in zip(jt.joint_names, jt.points[0].positions))
        if error > max(.03, tolerance * 2):
            raise RuntimeError('Robot moved since planning (%.4f rad). Plan the custom trajectory again.' % error)
        rospy.loginfo('Executing %s', action['file'])
        try:
            jt.header.stamp = rospy.Time(0)
            if not group.execute(trajectory, wait=True):
                raise RuntimeError('Execution failed: ' + action['file'])
        finally:
            group.stop()
            group.clear_pose_targets()


def save_sequence(run, sequence, config, planned):
    task = copy.deepcopy(sequence)
    task.update(stages=stages(sequence), planning=copy.deepcopy(config['planning']))
    (run / 'task.json').write_text(json.dumps(task, indent=2) + '\n')
    for action, trajectory, target in planned:
        with (run / action['group'] / action['file']).open('wb') as stream:
            pickle.dump(trajectory, stream)
        if 'target' in action:
            with (run / 'hand' / action['target']).open('wb') as stream:
                pickle.dump(target, stream)


def load_recording(run, robot, groups):
    task = load_task(run)
    planned = []
    for item, stage in zip(task['tasks'], task['stages']):
        for action in stage['actions']:
            backend = action['group']
            target = target_for_group(robot, groups[backend], item[backend])
            with (run / backend / action['file']).open('rb') as stream:
                trajectory = pickle.load(stream)
            jt = trajectory.joint_trajectory
            names = list(jt.joint_names)
            if not names or len(names) != len(set(names)) or set(names) != set(target) or not jt.points:
                raise ValueError('Saved custom joints/path do not match ' + action['file'])
            previous = -1
            for point in jt.points:
                stamp = point.time_from_start.to_sec()
                if (len(point.positions) != len(names) or not all(math.isfinite(v) for v in point.positions)
                        or not math.isfinite(stamp) or stamp < 0 or stamp < previous):
                    raise ValueError('Invalid custom trajectory: ' + action['file'])
                previous = stamp
            if any(abs(target[n] - v) > task['planning']['joint_tolerance'] + 1e-6
                   for n, v in zip(names, jt.points[-1].positions)):
                raise ValueError('Saved custom path does not reach target: ' + action['file'])
            if 'target' in action:
                with (run / 'hand' / action['target']).open('rb') as stream:
                    if target_for_group(robot, groups['hand'], pickle.load(stream)) != target:
                        raise ValueError('Custom hand target does not match its task')
            planned.append((action, trajectory, target))
    return task, planned


def replay(run, robot, groups, step=False, execute=False):
    import arm_replay
    import hand_replay
    from replay import is_simulation_replay, simulation_replay_speed
    from sim_contacts import disable_replay_collision_checks
    task, planned = load_recording(run, robot, groups)
    print('Custom trajectory checked: %s (%d tasks).' % (run.name, len(task['tasks'])), flush=True)
    if not execute:
        return
    restore = disable_replay_collision_checks(robot, protect_table=not is_simulation_replay())
    try:
        speed = simulation_replay_speed()
        position = 0
        for number, stage in enumerate(task['stages'], 1):
            label = 'Task %d/%d: %s' % (number, len(task['stages']), stage['name'])
            if step and input('Enter to execute %s (q to quit): ' % label).strip().lower() == 'q':
                return
            rospy.loginfo(label)
            for _ in stage['actions']:
                if rospy.is_shutdown():
                    raise RuntimeError('ROS shut down; sequence stopped')
                action, trajectory, target = planned[position]
                position += 1
                filename = action['file']
                if action['group'] == 'arm':
                    trajectory.joint_trajectory.header.stamp = rospy.Time(0)
                    arm_replay.execute_selected(groups['arm'], robot, {filename: trajectory}, filename, True,
                                                speed=speed, check_collisions=not is_simulation_replay())
                else:
                    hand_replay.plan_and_execute_target(groups['hand'], robot, target, filename, speed=speed)
    finally:
        try:
            for group in groups.values():
                group.stop()
        finally:
            restore()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('capture', 'plan'))
    parser.add_argument('--source', choices=('current', 'rviz'), default='rviz')
    parser.add_argument('--sequence')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('custom_trajectory', anonymous=True)
    run, successful = None, False
    try:
        require_simulation()
        robot = moveit_commander.RobotCommander()
        groups = {n: moveit_commander.MoveGroupCommander('right_' + n) for n in ('arm', 'hand')}
        if args.command == 'capture':
            # roscpp also writes to stdout. Emit the entire JSON record in one
            # write with its own line boundary so its buffered logs cannot split it.
            record = '\nCUSTOM_STATE_JSON=' + json.dumps(capture(robot, groups, args.source)) + '\n'
            os.write(sys.stdout.fileno(), record.encode())
            return
        if not args.sequence:
            raise ValueError('Provide --sequence PATH')
        sequence = validate_sequence(json.loads(Path(args.sequence).read_text()))
        config = load_config()
        configure_groups(groups, config['planning'])
        planned = plan_sequence(robot, groups, sequence)
        run = create_run(archive_root(), source=TASK_VERSION, task_version=TASK_VERSION, name=sequence['name'])
        save_sequence(run, sequence, config, planned)
        if args.execute:
            execute_plans(groups, planned, config['planning']['joint_tolerance'])
        successful = True
        rospy.loginfo('Custom trajectory saved: %s', run)
    finally:
        try:
            if run is not None:
                finish_run(run, successful)
        finally:
            moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
