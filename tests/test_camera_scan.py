"""Camera-frame geometry, branch search and execution guards; no robot motion."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from tf.transformations import quaternion_matrix
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
import camera_scan
from camera_views import camera_targets, settings, TASK_KIND
from project_config import load_config
from task_definition import TASK_VERSION
from trajectory_store import create_run, finish_run, replay_run_choices, resolve_run


class CameraGeometryTest(unittest.TestCase):
    def test_mount_adjustment_keeps_the_exact_original_camera_targets(self):
        config = load_config()
        self.assertEqual(settings(config)['distance'], .4)
        self.assertEqual(settings({})['distance'], .4)
        original = {**config, 'camera_scan': {**config.get('camera_scan', {}),
                    'distance': .4, 'mount_translation': [0, -.06, 0]}}
        expected = camera_targets(original)
        actual = camera_targets(config)
        for before, after in zip(expected['views'], actual['views']):
            np.testing.assert_array_equal(before['camera_optical_to_world'],
                                          after['camera_optical_to_world'])
            self.assertNotEqual(before['flange_pose'], after['flange_pose'])

    def test_defmapsim_optical_poses_and_flange_transform(self):
        config = load_config()
        # Fixed fixture independent of the user's current placement settings.
        config['object']['pose'] = [.41, 0, .034472, np.pi/2, 0, 0]
        config['object']['grasp_z_offset'] = .015
        config['camera_scan'] = {'roll_degrees': 0, 'distance': .4}
        result = camera_targets(config)
        expected = [[.7559731502, -.0364823601], [.4833226376, -.3417535963],
                    [.1087387972, -.1767815495], [.1498837648, .2304480189],
                    [.5498965937, .3171576865]]
        for view, xy in zip(result['views'], expected):
            camera = np.array(view['camera_optical_to_world'])
            np.testing.assert_allclose(camera[:3, 3], xy+[.248959126], atol=1e-9)
            np.testing.assert_allclose(camera[:3, :3].T.dot(camera[:3, :3]), np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(camera[:3, :3]), 1)
            pose = view['flange_pose']
            flange = quaternion_matrix([pose[k] for k in ('qx', 'qy', 'qz', 'qw')])
            flange[:3, 3] = [pose[k] for k in ('x', 'y', 'z')]
            np.testing.assert_allclose(flange.dot(result['flange_from_optical']), camera, atol=1e-10)

    def test_mount_changes_flange_but_not_requested_camera_views(self):
        config = load_config()
        first = camera_targets(config)
        config['camera_scan'] = {'mount_translation': [0, -.1, 0],
                                 'mount_forward': [.9949708551, 0, -.1001648512],
                                 'roll_degrees': settings(config)['roll_degrees']}
        second = camera_targets(config)
        for a, b in zip(first['views'], second['views']):
            np.testing.assert_allclose(a['camera_optical_to_world'], b['camera_optical_to_world'])
            self.assertNotEqual(a['flange_pose'], b['flange_pose'])
        config['object']['grasp_z_offset'] += .02
        raised = camera_targets(config)
        for a, b in zip(second['views'], raised['views']):
            self.assertAlmostEqual(b['flange_pose']['z']-a['flange_pose']['z'], .02)

    def test_half_turn_preserves_camera_position_and_aim(self):
        config = load_config()
        config['camera_scan'] = {'roll_degrees': 0}
        original = camera_targets(config)
        config['camera_scan']['roll_degrees'] = 180
        rolled = camera_targets(config)
        self.assertEqual(settings({})['roll_degrees'], 180)
        for before, after in zip(original['views'], rolled['views']):
            a, b = (np.array(v['camera_optical_to_world']) for v in (before, after))
            np.testing.assert_allclose(b[:3, 3], a[:3, 3], atol=1e-12)
            np.testing.assert_allclose(b[:3, 2], a[:3, 2], atol=1e-12)
            np.testing.assert_allclose(b[:3, :2], -a[:3, :2], atol=1e-12)
            self.assertNotEqual(before['flange_pose'], after['flange_pose'])

    def test_bad_calibration_and_budgets_rejected(self):
        for supplied in ({'mount_forward': [0, 0, 1]}, {'distance': 0},
                         {'ik_attempts': True}, {'max_path_attempts': 0},
                         {'pause_at_view': -1}, {'mount_up': [float('nan'), 0, 1]},
                         {'roll_degrees': float('nan')}, {'fist_joint_angles': {}},
                         {'fist_joint_angles': None}):
            with self.subTest(supplied=supplied), self.assertRaises(ValueError):
                settings({'camera_scan': supplied})


class CameraSequenceTest(unittest.TestCase):
    def test_failed_planner_result_is_not_used_even_with_partial_points(self):
        planner = camera_scan.ScanPlanner.__new__(camera_scan.ScanPlanner)
        planner.options = {'max_path_attempts': 1}
        planner.path_attempts = 0
        planner.robot, planner.check = Mock(), Mock()
        planner.check.return_value.valid = True
        group = Mock()
        group.plan.return_value = (False, self.action()['trajectory'], 0, None)
        with patch.object(camera_scan.rospy, 'is_shutdown', return_value=False), \
                patch.object(camera_scan, 'target_for_group', return_value={}), \
                patch.object(camera_scan, 'with_joints', return_value=Mock()):
            self.assertIsNone(planner.motion(group, Mock(), {}, 'failed.pkl', 'arm'))
        group.execute.assert_not_called()

    def test_failed_later_view_backtracks_to_another_configuration(self):
        planner = camera_scan.ScanPlanner.__new__(camera_scan.ScanPlanner)
        planner.geometry = {'views': [{'index': i, 'azimuth_degrees': i*72} for i in range(5)]}
        planner.arm, planner.robot = Mock(), Mock()
        planner.config = {'robot': {'home_joints': {'home': 0}}}
        planner.ik_goals = lambda state, view: ([{'branch': 'dead'}, {'branch': 'good'}]
            if view['index'] == 0 else [] if state == 'dead' else [{'branch': 'good'}])
        planner.motion = Mock(side_effect=lambda group, state, goal, filename, backend, view=None:
            {'file': filename, 'end_state': goal.get('branch', state), 'view': view})
        with patch.object(camera_scan, 'nearest_goal', return_value={'home': 0}):
            actions = planner.tour('start')
        self.assertEqual([a['view'] for a in actions[:5]], list(range(5)))
        self.assertEqual(actions[-1]['file'], '07_Return_home.pkl')
        self.assertEqual(planner.motion.call_count, 7)  # One failed branch, then six motions.

    def test_plan_all_closes_fist_before_planning_arm_motions(self):
        from moveit_msgs.msg import RobotState
        planner = camera_scan.ScanPlanner.__new__(camera_scan.ScanPlanner)
        planner.arm, planner.hand, planner.robot = Mock(), Mock(), Mock()
        planner.config = {'robot': {'home_joints': {'arm_joint': .2}}}
        planner.options = {'fist_joint_angles': {'rh_FFJ3': 1.5}}
        planner.geometry = {'views': [{'index': i, 'azimuth_degrees': i*72} for i in range(5)]}
        state = RobotState()
        state.joint_state.name = ['arm_joint', 'rh_FFJ3']
        state.joint_state.position = [.1, .4]
        planner.robot.get_current_state.return_value = state
        def ik_goals(state, view):
            self.assertEqual(state.joint_state.position[1], 1.5)
            return [{'arm_joint': view['index']*.1}]
        planner.ik_goals = ik_goals

        def motion(group, start, goal, filename, backend, view=None):
            if backend == 'hand':
                self.assertIs(group, planner.hand)
                self.assertEqual(planner.motion.call_count, 1)
                self.assertEqual(start.joint_state.position[1], .4)
            else:
                self.assertIs(group, planner.arm)
                self.assertEqual(start.joint_state.position[1], 1.5)
            return {'file': filename, 'group': backend, 'view': view,
                    'end_state': camera_scan.with_joints(start, goal)}

        planner.motion = Mock(side_effect=motion)
        with patch.object(camera_scan, 'nearest_goal', side_effect=lambda robot, state, goal: goal):
            actions = planner.plan_all()
        self.assertEqual(len(actions), 8)
        self.assertEqual([a['view'] for a in actions], [None, None, 0, 1, 2, 3, 4, None])
        self.assertEqual([a['file'] for a in actions], ['00_Close_fist.pkl', '01_Start_at_home.pkl',
            '02_View_000.pkl', '03_View_072.pkl', '04_View_144.pkl',
            '05_View_216.pkl', '06_View_288.pkl', '07_Return_home.pkl'])
        self.assertEqual(actions[-1]['end_state'].joint_state.position[1], 1.5)

    def test_failed_fist_preparation_stops_before_arm_planning(self):
        planner = camera_scan.ScanPlanner.__new__(camera_scan.ScanPlanner)
        planner.robot, planner.hand, planner.tour = Mock(), Mock(), Mock()
        planner.options = {'fist_joint_angles': {'rh_FFJ3': 1.5}}
        planner.motion = Mock(return_value=None)
        with self.assertRaisesRegex(RuntimeError, 'fist posture'):
            planner.plan_all()
        planner.motion.assert_called_once()
        planner.tour.assert_not_called()

    def test_archive_records_prepared_fist_instead_of_initial_hand_state(self):
        from moveit_msgs.msg import RobotState
        state = RobotState()
        state.joint_state.name = ['arm_joint', 'rh_FFJ3']
        state.joint_state.position = [.1, 1.5]
        planner = Mock()
        planner.options, planner.geometry = {}, {}
        planner.config = {'robot': {}, 'object': {}, 'grasp': {'joint_angles': {'rh_FFJ3': .4}}}
        action = self.action()
        action.update(group='hand', file='00_Close_fist.pkl', view=None, end_state=state)
        action['trajectory'].joint_trajectory.joint_names = ['rh_FFJ3']
        with tempfile.TemporaryDirectory() as folder:
            run = create_run(folder, task_version=TASK_KIND, grasp_file='camera_views.json')
            manifest = camera_scan.save_scan(run, planner, [action])
            self.assertEqual(manifest['hand_joint_angles'], {'rh_FFJ3': 1.5})
            self.assertTrue((run / 'hand' / '00_Close_fist.pkl').is_file())

    def test_camera_archive_cannot_be_selected_as_grasp_replay(self):
        with tempfile.TemporaryDirectory() as folder:
            run = create_run(folder, task_version=TASK_KIND, grasp_file='camera_views.json')
            finish_run(run, True)
            self.assertFalse(replay_run_choices(folder)[0]['replayable'])
            with self.assertRaises(ValueError):
                resolve_run(folder, 'latest', task_version=TASK_VERSION)

    def action(self):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['arm_joint']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[.1]),
                                               JointTrajectoryPoint(positions=[.2])]
        return {'group': 'arm', 'trajectory': trajectory, 'file': 'view.pkl', 'view': 0}

    def test_start_drift_prevents_execution(self):
        group = Mock()
        group.get_active_joints.return_value = ['arm_joint']
        group.get_current_joint_values.return_value = [.3]
        with patch.object(camera_scan.rospy, 'is_shutdown', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'start changed'):
                camera_scan.execute_scan(None, {'arm': group}, [self.action()], 1)
        group.execute.assert_not_called()

    def test_execute_failure_stops_and_does_not_continue(self):
        group = Mock()
        group.get_active_joints.return_value = ['arm_joint']
        group.get_current_joint_values.return_value = [.1]
        group.execute.return_value = False
        with patch.object(camera_scan.rospy, 'is_shutdown', return_value=False), \
                patch.object(camera_scan.rospy, 'sleep') as pause:
            with self.assertRaisesRegex(RuntimeError, 'execution failed'):
                camera_scan.execute_scan(None, {'arm': group}, [self.action(), self.action()], 1)
        group.execute.assert_called_once()
        group.stop.assert_called_once()
        pause.assert_not_called()

    def ik_fixture(self):
        from moveit_msgs.msg import RobotState
        planner = camera_scan.ScanPlanner.__new__(camera_scan.ScanPlanner)
        planner.arm, planner.robot, planner.solve, planner.check = Mock(), Mock(), Mock(), Mock()
        planner.arm.get_active_joints.return_value = ['arm_joint']
        planner.arm.get_name.return_value = 'right_arm'
        planner.robot.get_joint.return_value.bounds.return_value = [-np.pi, np.pi]
        planner.options = {'ik_attempts': 2, 'ik_timeout': .01, 'distance': .4, 'roll_degrees': 180}
        state = RobotState()
        state.joint_state.name = ['arm_joint', 'rh_FFJ3']
        state.joint_state.position = [0.0, 1.5]
        planner.solve.return_value.error_code.val = 1
        planner.solve.return_value.solution.joint_state.name = ['arm_joint']
        planner.solve.return_value.solution.joint_state.position = [.5]
        view = {'index': 0, 'azimuth_degrees': 0,
                'flange_pose': dict(x=.75, y=.02, z=.25, qx=0, qy=0, qz=0, qw=1)}
        return planner, state, view

    def test_no_ik_reports_geometry_without_claiming_collisions(self):
        planner, state, view = self.ik_fixture()
        planner.solve.return_value.error_code.val = -31
        with patch.object(camera_scan.rospy, 'is_shutdown', return_value=False), \
                patch.object(camera_scan.rospy, 'logwarn') as warning, \
                patch.object(camera_scan.rospy, 'loginfo') as info:
            self.assertEqual(planner.ik_goals(state, view), [])
        planner.check.assert_not_called()
        warning.assert_called_once()
        self.assertIn('No IK solution found', warning.call_args.args[0])
        self.assertEqual(warning.call_args.args[1:], (1, 0, 2, .4, 180))
        self.assertEqual(info.call_args.args[-2:], (0, 0))

    def test_collision_rejected_ik_reports_distinct_count_and_contact_pairs(self):
        planner, state, view = self.ik_fixture()
        planner.check.return_value.valid = False
        planner.check.return_value.contacts = [
            Mock(contact_body_1='rh_palm', contact_body_2='ground_plane_user'),
            Mock(contact_body_1='ground_plane_user', contact_body_2='rh_palm')]
        with patch.object(camera_scan.rospy, 'is_shutdown', return_value=False), \
                patch.object(camera_scan.rospy, 'logwarn') as warning, \
                patch.object(camera_scan.rospy, 'loginfo') as info:
            self.assertEqual(planner.ik_goals(state, view), [])
        planner.check.assert_called_once()
        self.assertEqual(planner.check.call_args.kwargs['robot_state'].joint_state.position[1], 1.5)
        warning.assert_called_once()
        self.assertIn('collide', warning.call_args.args[0])
        self.assertEqual(warning.call_args.args[-1], [('ground_plane_user', 'rh_palm')])
        self.assertEqual(info.call_args.args[-2:], (1, 0))


if __name__ == '__main__':
    unittest.main()
