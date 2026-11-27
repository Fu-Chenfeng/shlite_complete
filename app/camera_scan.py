#!/usr/bin/env python3
"""Plan fist preparation and a five-view camera tour back home, in simulation."""
import argparse
import copy
import json
import math
import os
import pickle
import random
import sys

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetStateValidity

from camera_views import TASK_KIND, camera_targets, settings
from joint_targets import target_for_group
from project_config import load_config
from sim_contacts import stage_one_contacts
from trajectory_store import archive_root, create_run, finish_run


def with_joints(state, values):
    result = copy.deepcopy(state)
    js = result.joint_state
    js.position = [values.get(n, v) for n, v in zip(js.name, js.position)]
    return result


def endpoint(state, trajectory):
    jt = trajectory.joint_trajectory
    return with_joints(state, dict(zip(jt.joint_names, jt.points[-1].positions)))


def nearest_goal(robot, state, goal):
    current = dict(zip(state.joint_state.name, state.joint_state.position))
    result = {}
    for name, value in goal.items():
        lo, hi = robot.get_joint(name).bounds()
        values = [value+2*math.pi*k for k in range(-2,3) if lo <= value+2*math.pi*k <= hi]
        if not values:
            raise ValueError('Camera goal outside joint limits: '+name)
        result[name] = min(values, key=lambda v: abs(v-current[name]))
    return result


class ScanPlanner:
    def __init__(self, robot, arm, hand, config):
        self.robot, self.arm, self.hand, self.config = robot, arm, hand, config
        self.options = settings(config)
        self.geometry = camera_targets(config)
        self.path_attempts = 0
        if arm.get_end_effector_link() != 'ra_flange':
            raise RuntimeError('Camera mounting calibration requires arm end effector ra_flange')
        for service in ('/compute_ik', '/check_state_validity'):
            rospy.wait_for_service(service, timeout=10)
        self.solve = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.check = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        for group, prefix in ((arm, 'arm'), (hand, 'hand')):
            group.set_planning_time(self.options['planning_time'])
            group.set_num_planning_attempts(config['planning']['planning_attempts'])
            group.set_max_velocity_scaling_factor(config['planning'][prefix+'_velocity'])
            group.set_max_acceleration_scaling_factor(config['planning'][prefix+'_acceleration'])
            # Tight arm endpoints preserve the requested optical pose after FK.
            group.set_goal_joint_tolerance(1e-5 if prefix == 'arm' else 0.003)
        arm.set_pose_reference_frame('world')

    def motion(self, group, state, goal, filename, backend, view=None):
        if rospy.is_shutdown():
            raise RuntimeError('Camera planning interrupted')
        if self.path_attempts >= self.options['max_path_attempts']:
            raise RuntimeError('Camera path search budget exhausted; no motion started')
        self.path_attempts += 1
        goal = target_for_group(self.robot, group, goal)
        target_state = with_joints(state, goal)
        validity = self.check(robot_state=target_state, group_name='')
        if not validity.valid:
            rospy.logwarn('%s target invalid: %s', filename,
                          sorted(set((c.contact_body_1,c.contact_body_2) for c in validity.contacts)))
            return None
        group.clear_pose_targets()
        group.set_start_state(state)
        group.set_joint_value_target(goal)
        result = group.plan()
        if isinstance(result, tuple) and not result[0]:
            return None
        trajectory = result[1] if isinstance(result, tuple) else result
        if not trajectory or not trajectory.joint_trajectory.points:
            return None
        return {'file': filename, 'group': backend, 'target': goal, 'view': view,
                'trajectory': trajectory, 'end_state': endpoint(state, trajectory)}

    def ik_goals(self, state, view):
        names = self.arm.get_active_joints()
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        seen, valid, contacts = [], [], set()
        rng = random.Random(1211)
        values = view['flange_pose']
        target = Pose()
        target.position.x, target.position.y, target.position.z = [values[k] for k in ('x','y','z')]
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = [values[k] for k in ('qx','qy','qz','qw')]
        for attempt in range(self.options['ik_attempts']):
            if rospy.is_shutdown():
                raise RuntimeError('Camera IK search interrupted')
            req = GetPositionIKRequest()
            ik = req.ik_request
            ik.group_name = self.arm.get_name()
            ik.ik_link_name = 'ra_flange'
            ik.robot_state = copy.deepcopy(state)
            if attempt:
                seed = {}
                for name in names:
                    lo, hi = self.robot.get_joint(name).bounds()
                    seed[name] = rng.uniform(max(lo,-math.pi), min(hi,math.pi))
                ik.robot_state = with_joints(state, seed)
            ik.pose_stamped.header.frame_id = 'world'
            ik.pose_stamped.pose = target
            ik.timeout = rospy.Duration(self.options['ik_timeout'])
            ik.avoid_collisions = False  # Every solution is explicitly checked below.
            answer = self.solve(req)
            if answer.error_code.val != 1:
                continue
            joints = dict(zip(answer.solution.joint_state.name, answer.solution.joint_state.position))
            goal = nearest_goal(self.robot, state, {n:joints[n] for n in names})
            if any(max(abs(math.atan2(math.sin(goal[n]-g[n]),math.cos(goal[n]-g[n]))) for n in names)<.01 for g in seen):
                continue
            seen.append(goal)
            validity = self.check(robot_state=with_joints(state, goal), group_name='')
            if validity.valid:
                valid.append(goal)
            else:
                contacts.update(tuple(sorted((c.contact_body_1, c.contact_body_2)))
                                for c in validity.contacts)
        valid.sort(key=lambda g: sum((g[n]-current[n])**2 for n in names))
        rospy.loginfo('Camera view %d/5 (%d deg): %d distinct IK configurations, %d collision-free',
                      view['index']+1, view['azimuth_degrees'], len(seen), len(valid))
        if not seen:
            rospy.logwarn('No IK solution found for camera view %d/5 (%d deg) after %d attempts '
                          'at distance %.3f m and roll %.1f deg; check orbit distance and mount calibration',
                          view['index']+1, view['azimuth_degrees'], self.options['ik_attempts'],
                          self.options['distance'], self.options['roll_degrees'])
        elif not valid:
            rospy.logwarn('All IK configurations for camera view %d/5 (%d deg) collide; contact pairs: %s',
                          view['index']+1, view['azimuth_degrees'], sorted(contacts))
        return valid

    def tour(self, state, index=0):
        if index == 5:
            home = nearest_goal(self.robot, state, self.config['robot']['home_joints'])
            back = self.motion(self.arm, state, home, '07_Return_home.pkl', 'arm')
            return [back] if back else None
        view = self.geometry['views'][index]
        for goal in self.ik_goals(state, view):
            action = self.motion(self.arm, state, goal, '%02d_View_%03d.pkl' % (index+2,view['azimuth_degrees']), 'arm', index)
            if action is not None:
                remaining = self.tour(action['end_state'], index+1)
                if remaining is not None:
                    return [action, *remaining]
            rospy.logwarn('Trying another configuration for camera view %d', index+1)
        return None

    def plan_all(self):
        state = self.robot.get_current_state()
        fist = self.motion(self.hand, state, self.options['fist_joint_angles'],
                           '00_Close_fist.pkl', 'hand')
        if fist is None:
            raise RuntimeError('Cannot plan the camera fist posture; no motion started')
        state = fist['end_state']
        home = nearest_goal(self.robot, state, self.config['robot']['home_joints'])
        first = self.motion(self.arm, state, home, '01_Start_at_home.pkl', 'arm')
        if first is None:
            raise RuntimeError('Cannot plan to arm home before the camera scan; reset simulation first')
        rest = self.tour(first['end_state'])
        if rest is None:
            raise RuntimeError('No complete five-view and return-home path found; no motion started')
        return [fist, first, *rest]


def save_scan(run, planner, actions):
    joints = actions[0]['end_state'].joint_state
    manifest = {'kind': TASK_KIND, 'camera_scan': planner.options, 'geometry': planner.geometry,
                'robot': planner.config['robot'], 'object': planner.config['object'],
                'hand_joint_angles': {n:v for n,v in zip(joints.name, joints.position) if n.startswith('rh_')},
                'motion_executed': False, 'images_captured': False, 'actions': []}
    for action in actions:
        with (run/action['group']/action['file']).open('wb') as stream:
            pickle.dump(action['trajectory'], stream)
        manifest['actions'].append({k:v for k,v in action.items() if k not in ('trajectory','end_state')})
    (run/'camera_scan.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def execute_scan(robot, groups, actions, pause):
    for action in actions:
        if rospy.is_shutdown():
            raise RuntimeError('Camera execution interrupted')
        group = groups[action['group']]
        actual = dict(zip(group.get_active_joints(), group.get_current_joint_values()))
        jt = action['trajectory'].joint_trajectory
        error = max(abs(actual[n]-v) for n,v in zip(jt.joint_names, jt.points[0].positions))
        if error > 0.03:
            raise RuntimeError('Camera trajectory start changed (%.4f rad); plan again' % error)
        rospy.loginfo('Camera sequence: %s', action['file'])
        try:
            jt.header.stamp = rospy.Time(0)
            if not group.execute(action['trajectory'], wait=True):
                raise RuntimeError('Camera sequence execution failed: '+action['file'])
        finally:
            group.stop()
        if action['view'] is not None:
            rospy.loginfo('At camera view %d/5; no depth camera capture is connected', action['view']+1)
            rospy.sleep(pause)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Execute in simulation after the complete path is planned')
    args = parser.parse_args(rospy.myargv()[1:])
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_five_camera_views', anonymous=True)
    run = None
    success = False
    try:
        if os.environ.get('SESSION_MODE') != 'sim' or not rospy.get_param('/use_sim_time',False):
            raise RuntimeError('Camera scan planning/execution is available only in simulation')
        config = load_config()
        if not config['object']['enabled']:
            raise ValueError('Enable the target object before planning camera views')
        robot = moveit_commander.RobotCommander()
        groups = {n:moveit_commander.MoveGroupCommander('right_'+n) for n in ('arm', 'hand')}
        run = create_run(archive_root(), source=TASK_KIND, task_version=TASK_KIND, grasp_file='camera_views.json')
        rospy.loginfo('Saving camera tour to %s', run)
        with stage_one_contacts(robot, config):
            planner = ScanPlanner(robot, groups['arm'], groups['hand'], config)
            from camera_visualization import publish_geometry
            scene_publisher = publish_geometry(planner.geometry)
            actions = planner.plan_all()
            manifest = save_scan(run, planner, actions)
            rospy.loginfo('Complete five-view sequence and return home planned and saved')
            if args.execute:
                execute_scan(robot, groups, actions, planner.options['pause_at_view'])
                manifest['motion_executed'] = True
                (run/'camera_scan.json').write_text(json.dumps(manifest, indent=2)+'\n')
            success = True
    finally:
        try:
            if run is not None:
                finish_run(run, success)
        finally:
            moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
