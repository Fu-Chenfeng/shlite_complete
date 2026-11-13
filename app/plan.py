#!/usr/bin/env python3
"""Plan, execute and save the configured five-stage grasp task in simulation."""
import copy
import math
import os
import pickle
import random
import sys

import moveit_commander
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetStateValidity

from grasp_pose import flange_pose, palm_pose, lift_endpoint, lift_waypoints
from joint_targets import target_for_group, approach_hand_target
from project_config import load_config, grasp_target
from sim_contacts import configure_hand_object_contacts, stage_one_contacts
from task_definition import TASK_VERSION, STAGES, save_task
from trajectory_store import archive_root, create_run, finish_run

PROJECT_CONFIG = load_config()
PROJECT_CONFIG['grasp'] = grasp_target(PROJECT_CONFIG)
TRAJECTORY_ROOT = str(archive_root())
ARM_TRAJ_DIR = None
HAND_TRAJ_DIR = None


def start_recording():
    global ARM_TRAJ_DIR, HAND_TRAJ_DIR
    grasp = PROJECT_CONFIG['grasp']
    run = create_run(TRAJECTORY_ROOT, task_version=TASK_VERSION,
                     grasp_file=grasp['capture_file'] if grasp.get('mode') == 'graspit' else None)
    save_task(run, PROJECT_CONFIG)
    ARM_TRAJ_DIR, HAND_TRAJ_DIR = str(run / 'arm'), str(run / 'hand')
    rospy.loginfo('Saving five-stage task to %s (Asia/Tokyo)', run)
    return run


def save_traj(traj, filename, group_name):
    if ARM_TRAJ_DIR is None or HAND_TRAJ_DIR is None:
        start_recording()
    directory = ARM_TRAJ_DIR if group_name == 'arm' else HAND_TRAJ_DIR
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, filename), 'wb') as stream:
        pickle.dump(traj, stream)
    rospy.loginfo('[%s] saved %s', group_name, filename)


def save_hand_target_joints(joints, filename):
    if ARM_TRAJ_DIR is None or HAND_TRAJ_DIR is None:
        start_recording()
    os.makedirs(HAND_TRAJ_DIR, exist_ok=True)
    with open(os.path.join(HAND_TRAJ_DIR, filename), 'wb') as stream:
        pickle.dump(dict(joints), stream)


def plan(group):
    result = group.plan()
    return result[1] if isinstance(result, tuple) else result


def pose(values):
    target = Pose()
    target.position.x, target.position.y, target.position.z = (values[k] for k in ('x', 'y', 'z'))
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = (
        values[k] for k in ('qx', 'qy', 'qz', 'qw'))
    return target


def execute_and_save(group, trajectory, action):
    if not trajectory or not trajectory.joint_trajectory.points:
        raise RuntimeError(action['file'] + ' planning failed')
    save_traj(trajectory, action['file'], action['group'])
    try:
        if not group.execute(trajectory, wait=True):
            raise RuntimeError(action['file'] + ' execute failed')
    finally:
        group.stop()
        group.clear_pose_targets()


def execute_pose(arm, pose_dict, action):
    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose(pose_dict))
    execute_and_save(arm, plan(arm), action)


def execute_hand(hand, target, action):
    from hand_replay import force_trajectory_start_to_current
    hand.stop()
    rospy.sleep(0.2)
    hand.set_start_state_to_current_state()
    hand.set_joint_value_target(target)
    save_hand_target_joints(target, action['target'])
    trajectory = plan(hand)
    if not trajectory or not trajectory.joint_trajectory.points:
        raise RuntimeError(action['file'] + ' planning failed')
    trajectory = force_trajectory_start_to_current(hand, trajectory, action['file'])
    execute_and_save(hand, trajectory, action)
    rospy.sleep(0.2)


def execute_home(arm, robot, target, action):
    current = dict(zip(arm.get_active_joints(), arm.get_current_joint_values()))
    target = dict(target)
    for name, value in target.items():
        equivalent = current[name] + math.atan2(math.sin(value - current[name]), math.cos(value - current[name]))
        lower, upper = robot.get_joint(name).bounds()
        if lower <= equivalent <= upper:
            target[name] = equivalent
    arm.set_start_state_to_current_state()
    arm.set_joint_value_target(target)
    execute_and_save(arm, plan(arm), action)


def hand_mount(arm, hand_link):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    transform = buffer.lookup_transform(arm.get_end_effector_link(), hand_link,
                                        rospy.Time(0), rospy.Duration(10)).transform
    translation, rotation = transform.translation, transform.rotation
    # Keep the listener alive until the lookup completes.
    del listener
    return [translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w]


def hand_grasp_flange_pose(arm, grasp):
    return flange_pose(grasp, *hand_mount(arm, grasp['hand_link']))


def stretch_time(trajectory, minimum_duration):
    points = trajectory.joint_trajectory.points
    duration = points[-1].time_from_start.to_sec()
    if duration <= 0:
        raise RuntimeError('Cartesian retiming returned no positive duration')
    scale = max(1.0, minimum_duration / duration)
    for point in points:
        point.time_from_start = rospy.Duration.from_sec(point.time_from_start.to_sec() * scale)
        point.velocities = [value / scale for value in point.velocities]
        point.accelerations = [value / (scale * scale) for value in point.accelerations]
    return trajectory


def verify_lift_endpoint(arm, robot, approach):
    """Reject an unreachable configured lift before starting the task."""
    rospy.wait_for_service('/compute_ik', timeout=10)
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm.get_name()
    request.ik_request.ik_link_name = arm.get_end_effector_link()
    request.ik_request.robot_state = robot.get_current_state()
    request.ik_request.pose_stamped.header.frame_id = PROJECT_CONFIG['grasp']['frame']
    distance = PROJECT_CONFIG['planning']['lift_distance']
    request.ik_request.pose_stamped.pose = pose(lift_endpoint(
        approach, PROJECT_CONFIG['grasp'], PROJECT_CONFIG['planning']))
    request.ik_request.timeout = rospy.Duration(2)
    # This is only an IK reachability probe. Stage 4 checks the entire path for collisions.
    request.ik_request.avoid_collisions = False
    result = rospy.ServiceProxy('/compute_ik', GetPositionIK)(request)
    if result.error_code.val != 1:
        raise RuntimeError('The configured grasp cannot reach the %.3f m lift endpoint with the requested orientation '
                           '(IK error %d). Adjust grasp.translation, grasp.orientation or planning.lift_distance.' %
                           (distance, result.error_code.val))


def verify_approach_endpoint(arm, robot, approach, hand_target=None):
    """Find distinct collision-free IK goals using multiple starting guesses."""
    for service in ('/compute_ik', '/check_state_validity'):
        rospy.wait_for_service(service, timeout=10)
    solve = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    check = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    state = robot.get_current_state()
    js = state.joint_state
    js.position = [((hand_target or {}).get(n, v)) for n, v in zip(js.name, js.position)]
    names = arm.get_active_joints()
    current = dict(zip(js.name, js.position))
    bounds = {n: robot.get_joint(n).bounds() for n in names}
    attempts = PROJECT_CONFIG['planning'].get('approach_ik_attempts', 64)
    timeout = PROJECT_CONFIG['planning'].get('approach_ik_timeout', 0.1)
    rng = random.Random(1211)
    seen, candidates, pairs = [], [], set()
    for attempt in range(attempts):
        if rospy.is_shutdown():
            raise RuntimeError('Approach IK search interrupted')
        request = GetPositionIKRequest()
        ik = request.ik_request
        ik.group_name = arm.get_name()
        ik.ik_link_name = arm.get_end_effector_link()
        ik.robot_state = copy.deepcopy(state)
        if attempt:
            seed = {n: rng.uniform(max(lo, -math.pi), min(hi, math.pi))
                    for n, (lo, hi) in bounds.items()}
            ik.robot_state.joint_state.position = [seed.get(n, v) for n, v in zip(js.name, js.position)]
        ik.pose_stamped.header.frame_id = PROJECT_CONFIG['grasp']['frame']
        ik.pose_stamped.pose = pose(approach)
        ik.timeout = rospy.Duration(timeout)
        # Generate a candidate, then explicitly validate the entire robot below.
        ik.avoid_collisions = False
        result = solve(request)
        if result.error_code.val != 1:
            continue
        values = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        target = {}
        for name in names:
            value = values[name]
            # Choose the equivalent angle nearest the current joint within limits.
            lo, hi = bounds[name]
            equivalents = [value + 2 * math.pi * k for k in range(-2, 3)
                           if lo <= value + 2 * math.pi * k <= hi]
            target[name] = min(equivalents, key=lambda v: abs(v-current[name])) if equivalents else value
        target_for_group(robot, arm, target)
        if any(max(abs(math.atan2(math.sin(target[n]-old[n]), math.cos(target[n]-old[n])))
                   for n in names) < 0.01 for old in seen):
            continue
        seen.append(target)
        candidate = copy.deepcopy(state)
        candidate.joint_state.position = [target.get(n, v) for n, v in zip(js.name, js.position)]
        validity = check(robot_state=candidate, group_name='')
        if validity.valid:
            candidates.append(target)
        else:
            pairs.update('%s / %s' % (c.contact_body_1, c.contact_body_2) for c in validity.contacts)
    candidates.sort(key=lambda goal: sum((goal[n]-current[n])**2 for n in names))
    rospy.loginfo('Approach IK: %d guesses, %d distinct configurations, %d collision-free goals',
                  attempts, len(seen), len(candidates))
    if candidates:
        return candidates
    detail = ', '.join(sorted(pairs)) or 'no valid IK goal found within the configured search budget'
    raise RuntimeError('Approach blocked before stage 1: %s. Checked %d starting guesses; '
                       'this finite search does not prove the pose impossible. '
                       'Stage 1 always checks hand-object collisions, including when object.hand_collision is false.'
                       % (detail, attempts))


def execute_approach(arm, candidates, action):
    """Try a collision-checked path to each IK goal before executing one."""
    for index, target in enumerate(candidates, 1):
        if rospy.is_shutdown():
            raise RuntimeError('Approach planning interrupted')
        rospy.loginfo('Planning approach to IK candidate %d/%d', index, len(candidates))
        arm.clear_pose_targets()
        arm.set_start_state_to_current_state()
        arm.set_joint_value_target(target)
        trajectory = plan(arm)
        if trajectory and trajectory.joint_trajectory.points:
            execute_and_save(arm, trajectory, action)
            return
        rospy.logwarn('No approach path found to candidate %d; trying the next configuration', index)
    raise RuntimeError('Stage 1 approach planning failed for all %d collision-free IK goals' % len(candidates))


def cartesian_lift_waypoints(arm, flange):
    """Use the same palm translation/rotation for preflight and actual lift."""
    options = PROJECT_CONFIG['planning']
    if options.get('lift_align_gravity', False):
        mount_t, mount_q = hand_mount(arm, PROJECT_CONFIG['grasp']['hand_link'])
        start = palm_pose(flange, mount_t, mount_q)
        palms = lift_waypoints(start, options['lift_distance'], options['lift_step'],
                               PROJECT_CONFIG['grasp']['gravity_direction_palm'],
                               math.radians(options.get('lift_rotation_step_degrees', 2.0)))
        return [pose(flange_pose(palm, mount_t, mount_q)) for palm in palms]
    target = pose(flange)
    target.position.z += options['lift_distance']
    return [target]


def filter_lift_candidates(arm, robot, approach, candidates, grasp, tightened):
    """Reject grasp configurations whose closed-hand states or lift are blocked."""
    options = PROJECT_CONFIG['planning']
    points = cartesian_lift_waypoints(arm, approach)
    required = 1.0 if options.get('lift_align_gravity', False) else options['lift_min_fraction']
    rospy.wait_for_service('/check_state_validity', timeout=10)
    check = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    current = robot.get_current_state()
    accepted = []
    best_fraction = 0.0
    try:
        for index, goal in enumerate(candidates, 1):
            if rospy.is_shutdown():
                raise RuntimeError('Lift preflight interrupted')
            state = None
            for label, hand in (('closed', grasp), ('tightened', tightened)):
                state = copy.deepcopy(current)
                values = dict(hand)
                values.update(goal)
                js = state.joint_state
                js.position = [values.get(n, v) for n, v in zip(js.name, js.position)]
                validity = check(robot_state=state, group_name='')
                if not validity.valid:
                    pairs = sorted(set('%s / %s' % (c.contact_body_1, c.contact_body_2)
                                       for c in validity.contacts))
                    rospy.logwarn('Rejecting approach candidate %d: %s hand state invalid (%s)',
                                  index, label, ', '.join(pairs))
                    state = None
                    break
            if state is None:
                continue
            arm.set_start_state(state)
            trajectory, fraction = arm.compute_cartesian_path(
                points, options['lift_step'], 0.0, avoid_collisions=True)
            best_fraction = max(best_fraction, fraction)
            rospy.loginfo('Approach candidate %d/%d: lift path %.1f%%', index, len(candidates), 100*fraction)
            if trajectory and trajectory.joint_trajectory.points and fraction >= required:
                accepted.append(goal)
    finally:
        arm.set_start_state_to_current_state()
    if not accepted:
        raise RuntimeError('No tested approach configuration supports the configured lift before stage 1: '
                           'best lift path %.1f%% across %d candidates. Adjust the grasp or lift settings; '
                           'no task motion was started.' % (100*best_fraction, len(candidates)))
    rospy.loginfo('Keeping %d/%d approach configurations that support the lift', len(accepted), len(candidates))
    return accepted


def execute_cartesian_z_lift(arm, robot, action):
    options = PROJECT_CONFIG['planning']
    target = arm.get_current_pose().pose
    flange = dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                  (target.position.x, target.position.y, target.position.z,
                   target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w)))
    align = options.get('lift_align_gravity', False)
    waypoints = cartesian_lift_waypoints(arm, flange)
    if align:
        rospy.loginfo('Stage 4: smoothly align captured gravity with world -Z while lifting the palm')
    arm.set_start_state_to_current_state()
    trajectory, fraction = arm.compute_cartesian_path(
        waypoints, options['lift_step'], 0.0, avoid_collisions=True)
    required_fraction = 1.0 if align else options['lift_min_fraction']
    if not trajectory or not trajectory.joint_trajectory.points or fraction < required_fraction:
        raise RuntimeError('Stage 4 Cartesian lift planning failed: requested %.3f m, valid path %.1f%%. '
                           'Adjust grasp.translation, grasp.orientation or planning.lift_distance.' %
                           (options['lift_distance'], 100 * fraction))
    trajectory = arm.retime_trajectory(
        robot.get_current_state(), trajectory,
        velocity_scaling_factor=options.get('lift_velocity', options['arm_velocity']),
        acceleration_scaling_factor=options.get('lift_acceleration', options['arm_acceleration']))
    if not trajectory or not trajectory.joint_trajectory.points:
        raise RuntimeError('Stage 4 Cartesian lift retiming failed')
    execute_and_save(arm, stretch_time(trajectory, options['lift_min_duration']), action)


def execute_stages(robot):
    config = PROJECT_CONFIG
    options = config['planning']
    arm = moveit_commander.MoveGroupCommander('right_arm')
    hand = moveit_commander.MoveGroupCommander('right_hand')
    arm.set_pose_reference_frame(config['grasp']['frame'])
    for group, prefix in ((arm, 'arm'), (hand, 'hand')):
        group.set_max_velocity_scaling_factor(options[prefix + '_velocity'])
        group.set_max_acceleration_scaling_factor(options[prefix + '_acceleration'])
        group.set_planning_time(options['planning_time'])
        group.set_num_planning_attempts(options['planning_attempts'])
        group.set_goal_joint_tolerance(options['joint_tolerance'])
    # Validate every target before starting stage 1, including all-zero reset.
    home = target_for_group(robot, arm, config['robot']['home_joints'])
    grasp = target_for_group(robot, hand, config['grasp']['joint_angles'], fill_zero=True)
    tightened = target_for_group(robot, hand, config['grasp']['tightened_joint_angles'], fill_zero=True)
    for name, limit in config['grasp'].get('tightening_limited_joints', {}).items():
        rospy.logwarn('Stage 3 tightening limited for %s: requested %+.2f deg, applied %+.2f deg; '
                      'target %.2f deg is at the joint limit.', name, limit['requested_degrees'],
                      limit['applied_degrees'], limit['target_degrees'])
    zero = target_for_group(robot, hand, {}, fill_zero=True)
    prepared = approach_hand_target(robot, hand, grasp, options.get('approach_hand_fraction', 0.6),
                                    options['joint_tolerance'])
    approach = hand_grasp_flange_pose(arm, config['grasp'])
    with stage_one_contacts(robot, config):
        approach_candidates = verify_approach_endpoint(arm, robot, approach, prepared)
    approach_candidates = filter_lift_candidates(arm, robot, approach, approach_candidates, grasp, tightened)

    rospy.loginfo('Stage 1/5: prepare thumb base rh_THJ5=%.6f, rh_THJ4=%.6f rad',
                  prepared['rh_THJ5'], prepared['rh_THJ4'])
    rospy.loginfo('All finger J4 joints reach their full targets; remaining hand joints move %.0f%% '
                  'from current angles toward the grasp before arm motion.',
                  100 * options.get('approach_hand_fraction', 0.6))
    with stage_one_contacts(robot, config):
        execute_hand(hand, prepared, STAGES[0]['actions'][0])
        rospy.loginfo('Stage 1/5: approach the configured palm pose (hand-object collision checking enabled)')
        execute_approach(arm, approach_candidates, STAGES[0]['actions'][1])
    rospy.loginfo('Stage 2/5: close to grasp joint angles')
    execute_hand(hand, grasp, STAGES[1]['actions'][0])
    rospy.loginfo('Stage 3/5: apply tightened grasp joint angles')
    execute_hand(hand, tightened, STAGES[2]['actions'][0])
    rospy.loginfo('Stage 4/5: lift hand by %.3f m in world Z', options['lift_distance'])
    execute_cartesian_z_lift(arm, robot, STAGES[3]['actions'][0])
    rospy.loginfo('Stage 5/5: return arm home, then set every hand joint to zero')
    execute_home(arm, robot, home, STAGES[4]['actions'][0])
    execute_hand(hand, zero, STAGES[4]['actions'][1])


def execute_task():
    robot = moveit_commander.RobotCommander()
    restore = configure_hand_object_contacts(robot, PROJECT_CONFIG)
    try:
        execute_stages(robot)
    finally:
        restore()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_shadowhand_grasp_task', anonymous=True)
    if os.environ.get('SESSION_MODE') != 'sim' or not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Planning-and-saving workflow must run in the simulation session')
    run = start_recording()
    successful = False
    try:
        execute_task()
        successful = True
    finally:
        try:
            finish_run(run, successful)
        finally:
            moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
