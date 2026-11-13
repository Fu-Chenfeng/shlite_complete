"""Offline migration checks using real ROS message types, without a master."""
import importlib.util
import importlib.machinery
from pathlib import Path
import pickle
import os
import math
import subprocess
import sys
import tempfile
import unittest
from contextlib import nullcontext, contextmanager
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))
import plan
import replay
from trajectory_store import resolve_run
from task_definition import STAGES, TASK_VERSION, load_task
from recording_fixture import make_recording
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

loader = importlib.machinery.SourceFileLoader("robotctl", str(ROOT / "robotctl"))
spec = importlib.util.spec_from_loader(loader.name, loader)
ctl = importlib.util.module_from_spec(spec)
loader.exec_module(ctl)


class ProjectContractTest(unittest.TestCase):
    def setUp(self):
        # These contract tests have no ROS master; contact restoration has its
        # own service-level tests in test_sim_contacts.py.
        for module in (plan,):
            patcher = patch.object(module, 'stage_one_contacts', side_effect=lambda *args: nullcontext())
            patcher.start()
            self.addCleanup(patcher.stop)

    def run_planner_without_motion(self, root, task):
        with patch.dict(os.environ, {'SESSION_MODE': 'sim'}), \
                patch.object(plan, 'TRAJECTORY_ROOT', str(root)), \
                patch.object(plan, 'ARM_TRAJ_DIR', None), patch.object(plan, 'HAND_TRAJ_DIR', None), \
                patch.object(plan.rospy, 'init_node'), patch.object(plan.rospy, 'get_param', return_value=True), \
                patch.object(plan.moveit_commander, 'roscpp_initialize'), \
                patch.object(plan.moveit_commander, 'roscpp_shutdown'), \
                patch.object(plan, 'execute_task', side_effect=task):
            plan.main()

    def test_each_plan_groups_all_saved_files_in_a_new_grasp_named_run(self):
        def save():
            plan.save_traj(RobotTrajectory(), 'Arm_path.pkl', 'arm')
            plan.save_traj(RobotTrajectory(), 'Hand_path.pkl', 'hand')
            plan.save_hand_target_joints({'rh_THJ1': 0.2}, 'Hand_path_target_joints.pkl')
        with tempfile.TemporaryDirectory() as folder:
            self.run_planner_without_motion(folder, save)
            first = resolve_run(folder)
            self.assertEqual(first.name, Path(plan.PROJECT_CONFIG['grasp']['capture_file']).stem)
            expected = {'arm/Arm_path.pkl', 'hand/Hand_path.pkl',
                        'hand/Hand_path_target_joints.pkl'}
            self.assertEqual({str(p.relative_to(first)) for p in first.rglob('*.pkl')}, expected)
            first_bytes = {str(p.relative_to(first)): p.read_bytes() for p in first.rglob('*.pkl')}
            self.run_planner_without_motion(folder, save)
            second = resolve_run(folder)
            self.assertEqual(second.name, first.name + '_001')
            self.assertNotEqual(first, second)
            self.assertEqual({str(p.relative_to(first)): p.read_bytes() for p in first.rglob('*.pkl')}, first_bytes)
            self.assertFalse(Path(folder, 'arm').exists())
            self.assertFalse(Path(folder, 'hand').exists())

    def test_failed_plan_keeps_partial_files_without_replacing_latest_completed_run(self):
        def save():
            plan.save_traj(RobotTrajectory(), 'Arm_path.pkl', 'arm')
        def fail():
            save()
            raise RuntimeError('planning failed')
        with tempfile.TemporaryDirectory() as folder:
            self.run_planner_without_motion(folder, save)
            complete = resolve_run(folder)
            with self.assertRaisesRegex(RuntimeError, 'planning failed'):
                self.run_planner_without_motion(folder, fail)
            self.assertEqual(resolve_run(folder), complete)
            self.assertEqual(len(list(Path(folder).glob('*/arm/Arm_path.pkl'))), 2)

    def test_five_stages_include_home_before_zeroing_hand(self):
        self.assertEqual(len(STAGES), 5)
        self.assertEqual(replay.SEQUENCE, [
            ('hand', 'Stage_1_Prepare_thumb_base.pkl'),
            ('arm', 'Stage_1_Approach_grasp.pkl'),
            ('hand', 'Stage_2_Close_fingers.pkl'),
            ('hand', 'Stage_3_Tighten_grasp.pkl'),
            ('arm', 'Stage_4_Lift.pkl'),
            ('arm', 'Stage_5_Return_home.pkl'),
            ('hand', 'Stage_5_Open_hand.pkl'),
        ])

    def test_saved_ros_trajectory_and_joint_target_formats_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            arm, hand = Path(folder, 'arm'), Path(folder, 'hand')
            with patch.object(plan, 'ARM_TRAJ_DIR', str(arm)), patch.object(plan, 'HAND_TRAJ_DIR', str(hand)):
                trajectory = RobotTrajectory()
                trajectory.joint_trajectory.joint_names = ['ra_elbow_joint']
                trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.5])]
                plan.save_traj(trajectory, STAGES[0]['actions'][1]['file'], 'arm')
                path = arm / replay.SEQUENCE[1][1]
                self.assertEqual(pickle.loads(path.read_bytes()), trajectory)
                filename = STAGES[1]['actions'][0]['target']
                plan.save_hand_target_joints({'rh_THJ1': 0.2}, filename)
                self.assertEqual(pickle.loads((hand / filename).read_bytes()), {'rh_THJ1': 0.2})

    def test_replay_prompts_five_times_and_home_failure_prevents_hand_reset(self):
        groups = {'arm': Mock(), 'hand': Mock()}
        data = {name: RobotTrajectory() for _, name in replay.SEQUENCE}
        targets = {name: {'rh_THJ5': 0.0} for group, name in replay.SEQUENCE if group == 'hand'}
        events = []
        def arm_action(*args):
            events.append(args[3])
        def hand_action(*args):
            events.append(args[3])
        with patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(replay, 'thumb_base_target', side_effect=lambda robot, hand, angles, tolerance: angles), \
                patch.object(replay.arm_replay, 'execute_selected', side_effect=arm_action), \
                patch.object(replay.hand_replay, 'plan_and_execute_target', side_effect=hand_action), \
                patch('builtins.input', return_value='') as prompt:
            replay.execute_stages(groups, Mock(), data, targets, step=True)
        self.assertEqual(prompt.call_count, 5)
        self.assertEqual(events, [name for _, name in replay.SEQUENCE])
        def fail_home(*args):
            if args[3] == 'Stage_5_Return_home.pkl':
                raise RuntimeError('home failed')
        with patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(replay, 'thumb_base_target', side_effect=lambda robot, hand, angles, tolerance: angles), \
                patch.object(replay.arm_replay, 'execute_selected', side_effect=fail_home), \
                patch.object(replay.hand_replay, 'plan_and_execute_target') as hand:
            with self.assertRaisesRegex(RuntimeError, 'home failed'):
                replay.execute_stages(groups, Mock(), data, targets)
            self.assertEqual(hand.call_count, 3)

    def test_recording_uses_saved_hand_targets_after_config_changes(self):
        groups = {'arm': Mock(), 'hand': Mock()}
        groups['arm'].get_active_joints.return_value = list(plan.PROJECT_CONFIG['robot']['home_joints'])
        groups['hand'].get_active_joints.return_value = list(plan.PROJECT_CONFIG['grasp']['joint_angles'])
        robot = Mock()
        robot.get_joint.return_value.bounds.return_value = [-10, 10]
        with tempfile.TemporaryDirectory() as folder:
            run = make_recording(folder, robot, groups, plan.PROJECT_CONFIG)
            _, _, targets = replay.load_recording(run, groups, robot)
            self.assertEqual(targets['Stage_3_Tighten_grasp.pkl'], plan.PROJECT_CONFIG['grasp']['tightened_joint_angles'])
            self.assertTrue(all(value == 0 for value in targets['Stage_5_Open_hand.pkl'].values()))
            self.assertEqual(load_task(run)['version'], TASK_VERSION)

    def test_replay_prepares_only_thumb_base_using_live_finger_angles(self):
        groups = {'arm': Mock(), 'hand': Mock()}
        names = list(plan.PROJECT_CONFIG['grasp']['joint_angles'])
        groups['hand'].get_active_joints.return_value = names
        groups['hand'].get_current_joint_values.return_value = [0.2] * len(names)
        robot = Mock()
        robot.get_joint.return_value.bounds.return_value = [-10, 10]
        data = {name: RobotTrajectory() for _, name in replay.SEQUENCE}
        targets = {name: dict(plan.PROJECT_CONFIG['grasp']['joint_angles'])
                   for group, name in replay.SEQUENCE if group == 'hand'}
        for preparation in (('rh_THJ5',), ('rh_THJ5', 'rh_THJ4')):
            with self.subTest(preparation=preparation), \
                    patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                    patch.object(replay.arm_replay, 'execute_selected'), \
                    patch.object(replay.hand_replay, 'plan_and_execute_target') as execute:
                replay.execute_stages(groups, robot, data, targets,
                                      preparation_joints=preparation)
            prepared = execute.call_args_list[0].args[2]
            for name in preparation:
                self.assertEqual(prepared[name], targets['Stage_1_Prepare_thumb_base.pkl'][name])
            self.assertTrue(all(value == 0.2 for name, value in prepared.items() if name not in preparation))
            self.assertEqual(execute.call_args_list[1].args[2], targets['Stage_2_Close_fingers.pkl'])

        # New recordings replay their exact partial-close target, regardless of
        # live finger angles or subsequent config changes.
        saved = {name: 0.3 for name in names}
        targets['Stage_1_Prepare_thumb_base.pkl'] = saved
        with patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(replay.arm_replay, 'execute_selected'), \
                patch.object(replay.hand_replay, 'plan_and_execute_target') as execute, \
                patch.object(replay, 'thumb_base_target') as legacy:
            replay.execute_stages(groups, robot, data, targets, saved_preparation=True)
            legacy.assert_not_called()
            self.assertEqual(execute.call_args_list[0].args[2], saved)

    def test_failed_thumb_preparation_stops_before_arm_approach(self):
        data = {name: RobotTrajectory() for _, name in replay.SEQUENCE}
        targets = {name: {'rh_THJ5': 0.0} for group, name in replay.SEQUENCE if group == 'hand'}
        with patch.object(replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(replay, 'thumb_base_target', return_value={'rh_THJ5': 0.0}), \
                patch.object(replay.hand_replay, 'plan_and_execute_target', side_effect=RuntimeError('thumb failed')), \
                patch.object(replay.arm_replay, 'execute_selected') as arm:
            with self.assertRaisesRegex(RuntimeError, 'thumb failed'):
                replay.execute_stages({'arm': Mock(), 'hand': Mock()}, Mock(), data, targets)
        arm.assert_not_called()

    def test_blocked_approach_reports_contact_bodies_before_planning(self):
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        arm.get_name.return_value = 'right_arm'
        arm.get_end_effector_link.return_value = 'ra_flange'
        arm.get_active_joints.return_value = ['ra_joint']
        robot.get_joint.return_value.bounds.return_value = [-6.28, 6.28]
        state = RobotState()
        state.joint_state.name = ['rh_THJ5', 'rh_FFJ3', 'ra_joint']
        state.joint_state.position = [0.0, 0.2, 0.0]
        robot.get_current_state.return_value = state
        solver = Mock(side_effect=[Mock(error_code=Mock(val=-31)),
                                   Mock(error_code=Mock(val=1), solution=state)])
        validity = Mock(return_value=Mock(valid=False, contacts=[Mock(contact_body_1='rect_object__link',
                                                       contact_body_2='rh_thmiddle')]))
        def service(name, kind):
            return solver if name == '/compute_ik' else validity
        with patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'approach_ik_attempts': 2}), \
                patch.object(plan.rospy, 'ServiceProxy', side_effect=service):
            with self.assertRaisesRegex(RuntimeError, 'Approach blocked.*rect_object__link / rh_thmiddle'):
                plan.verify_approach_endpoint(arm, robot,
                    {'x': 0.3, 'y': 0, 'z': 0.2, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1},
                    {'rh_THJ5': -0.9})
        self.assertEqual(solver.call_args.args[0].ik_request.robot_state.joint_state.position[:2], [-0.9, 0.2])
        arm.plan.assert_not_called()
        arm.execute.assert_not_called()

    def test_approach_search_finds_alternative_and_keeps_prepared_hand(self):
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        arm.get_name.return_value = 'right_arm'
        arm.get_end_effector_link.return_value = 'ra_flange'
        arm.get_active_joints.return_value = ['ra_joint']
        robot.get_joint.return_value.bounds.return_value = [-6.28, 6.28]
        state = RobotState()
        state.joint_state.name = ['ra_joint', 'rh_THJ5']
        state.joint_state.position = [0.0, 0.0]
        robot.get_current_state.return_value = state
        responses = []
        for angle in (0.5, 1.0, 1.0 + 2 * math.pi):
            result = RobotState()
            result.joint_state.name = state.joint_state.name
            result.joint_state.position = [angle, 0.0]
            responses.append(Mock(error_code=Mock(val=1), solution=result))
        solver = Mock(side_effect=responses)
        validity = Mock(side_effect=[Mock(valid=False, contacts=[]), Mock(valid=True)])
        def service(name, kind):
            return solver if name == '/compute_ik' else validity
        with patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan.rospy, 'ServiceProxy', side_effect=service), \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'approach_ik_attempts': 3}):
            goals = plan.verify_approach_endpoint(arm, robot,
                {'x': 0.3, 'y': 0, 'z': 0.2, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1},
                {'rh_THJ5': -0.9})
        self.assertEqual(goals, [{'ra_joint': 1.0}])
        self.assertEqual(validity.call_count, 2)  # Equivalent turns are deduplicated.
        for call in validity.call_args_list:
            self.assertEqual(call.kwargs['group_name'], '')  # Whole robot checked.
            self.assertEqual(call.kwargs['robot_state'].joint_state.position[1], -0.9)
        arm.execute.assert_not_called()

    def test_approach_tries_next_goal_on_path_failure_but_not_execution_failure(self):
        arm = Mock()
        empty, good = RobotTrajectory(), RobotTrajectory()
        good.joint_trajectory.points = [JointTrajectoryPoint()]
        goals = [{'ra_joint': 0.5}, {'ra_joint': 1.0}]
        with patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan, 'plan', side_effect=[empty, good]), \
                patch.object(plan, 'execute_and_save') as execute:
            plan.execute_approach(arm, goals, {'file': 'approach.pkl'})
            execute.assert_called_once_with(arm, good, {'file': 'approach.pkl'})
        with patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan, 'plan', return_value=good) as planner, \
                patch.object(plan, 'execute_and_save', side_effect=RuntimeError('execution failed')):
            with self.assertRaisesRegex(RuntimeError, 'execution failed'):
                plan.execute_approach(arm, goals, {'file': 'approach.pkl'})
            self.assertEqual(planner.call_count, 1)
        with patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan, 'plan', return_value=empty), \
                patch.object(plan, 'execute_and_save') as execute:
            with self.assertRaisesRegex(RuntimeError, 'failed for all 2'):
                plan.execute_approach(arm, goals, {'file': 'approach.pkl'})
            execute.assert_not_called()

    def test_lift_preflight_rejects_partial_path_and_checks_tightened_state(self):
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        state = RobotState()
        state.joint_state.name = ['ra_joint', 'rh_joint']
        state.joint_state.position = [0.0, 0.0]
        robot.get_current_state.return_value = state
        good = RobotTrajectory()
        good.joint_trajectory.points = [JointTrajectoryPoint()]
        arm.compute_cartesian_path.side_effect = [(good, 0.68), (good, 1.0)]
        goals = [{'ra_joint': 0.5}, {'ra_joint': 1.0}]
        validity = Mock(return_value=Mock(valid=True))
        with patch.object(plan, 'cartesian_lift_waypoints', return_value=['waypoint']), \
                patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan.rospy, 'ServiceProxy', return_value=validity), \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': True}):
            selected = plan.filter_lift_candidates(arm, robot, {}, goals,
                                                  {'rh_joint': 0.4}, {'rh_joint': 0.6})
        self.assertEqual(selected, [goals[1]])
        self.assertEqual(state.joint_state.position, [0.0, 0.0])
        self.assertEqual([call.args[0].joint_state.position for call in arm.set_start_state.call_args_list],
                         [[0.5, 0.6], [1.0, 0.6]])
        self.assertEqual(validity.call_count, 4)
        for call in arm.compute_cartesian_path.call_args_list:
            self.assertTrue(call.kwargs['avoid_collisions'])
        arm.set_start_state_to_current_state.assert_called_once()
        arm.execute.assert_not_called()

    def test_lift_preflight_rejects_invalid_hand_and_stops_when_no_candidate_works(self):
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        state = RobotState()
        state.joint_state.name = ['ra_joint', 'rh_joint']
        state.joint_state.position = [0.0, 0.0]
        robot.get_current_state.return_value = state
        validity = Mock(return_value=Mock(valid=False, contacts=[]))
        with patch.object(plan, 'cartesian_lift_waypoints', return_value=['waypoint']), \
                patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'is_shutdown', return_value=False), \
                patch.object(plan.rospy, 'ServiceProxy', return_value=validity):
            with self.assertRaisesRegex(RuntimeError, 'No tested approach configuration.*before stage 1'):
                plan.filter_lift_candidates(arm, robot, {}, [{'ra_joint': 0.5}],
                                           {'rh_joint': 0.4}, {'rh_joint': 0.6})
        arm.compute_cartesian_path.assert_not_called()
        arm.execute.assert_not_called()
        arm.set_start_state_to_current_state.assert_called_once()

    def test_planner_executes_approach_close_tighten_lift_home_then_zero(self):
        arm, hand, robot = Mock(), Mock(), Mock()
        arm.get_active_joints.return_value = list(plan.PROJECT_CONFIG['robot']['home_joints'])
        hand.get_active_joints.return_value = list(plan.PROJECT_CONFIG['grasp']['joint_angles'])
        hand.get_current_joint_values.return_value = [0.1] * len(hand.get_active_joints.return_value)
        robot.get_joint.return_value.bounds.return_value = [-10, 10]
        events = []
        checking = False
        @contextmanager
        def scope(*args):
            nonlocal checking
            checking = True
            try:
                yield
            finally:
                checking = False
        def preflight(*args):
            self.assertTrue(checking)
            return []
        def lift_check(*args):
            self.assertFalse(checking)
            return []
        def record(*args):
            self.assertEqual(checking, args[-1]['file'].startswith('Stage_1_'))
            events.append(args)
        with patch.object(plan.moveit_commander, 'MoveGroupCommander', side_effect=[arm, hand]), \
                patch.object(plan, 'hand_grasp_flange_pose', return_value={'configured': 'palm'}), \
                patch.object(plan, 'stage_one_contacts', side_effect=scope), \
                patch.object(plan, 'verify_approach_endpoint', side_effect=preflight), \
                patch.object(plan, 'filter_lift_candidates', side_effect=lift_check), \
                patch.object(plan, 'execute_approach', side_effect=record), \
                patch.object(plan, 'execute_hand', side_effect=record), \
                patch.object(plan, 'execute_cartesian_z_lift', side_effect=record), \
                patch.object(plan, 'execute_home', side_effect=record):
            plan.execute_stages(robot)
        self.assertEqual([args[-1]['file'] for args in events], [name for _, name in replay.SEQUENCE])
        self.assertEqual(events[0][1]['rh_THJ5'], plan.PROJECT_CONFIG['grasp']['joint_angles']['rh_THJ5'])
        self.assertEqual(events[0][1]['rh_THJ4'], plan.PROJECT_CONFIG['grasp']['joint_angles']['rh_THJ4'])
        for name, value in events[0][1].items():
            goal = plan.PROJECT_CONFIG['grasp']['joint_angles'][name]
            expected = goal if name == 'rh_THJ5' or name.endswith('J4') else 0.1 + plan.PROJECT_CONFIG['planning'].get('approach_hand_fraction', 0.6) * (goal - 0.1)
            self.assertAlmostEqual(value, expected)
        self.assertEqual(events[2][1], plan.PROJECT_CONFIG['grasp']['joint_angles'])
        self.assertEqual(events[3][1], plan.PROJECT_CONFIG['grasp']['tightened_joint_angles'])
        self.assertEqual(events[5][2], plan.PROJECT_CONFIG['robot']['home_joints'])
        self.assertTrue(all(value == 0 for value in events[6][1].values()))


    def test_unreachable_lift_endpoint_is_reported_before_motion(self):
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        arm.get_name.return_value = 'right_arm'
        arm.get_end_effector_link.return_value = 'ra_flange'
        robot.get_current_state.return_value = RobotState()
        solver = Mock(return_value=Mock(error_code=Mock(val=-31)))
        with patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'ServiceProxy', return_value=solver), \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': False}):
            with self.assertRaisesRegex(RuntimeError, 'cannot reach the .* m lift endpoint'):
                plan.verify_lift_endpoint(arm, robot, {'x': 0.5, 'y': 0, 'z': 0.2,
                                                     'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1})
        request = solver.call_args.args[0]
        self.assertAlmostEqual(request.ik_request.pose_stamped.pose.position.z,
                               0.2 + plan.PROJECT_CONFIG['planning']['lift_distance'])
        arm.execute.assert_not_called()

    def test_home_uses_nearby_equivalent_wrist_angle_without_a_full_revolution(self):
        arm, robot = Mock(), Mock()
        arm.get_active_joints.return_value = ['ra_wrist_3_joint']
        arm.get_current_joint_values.return_value = [3.141]
        robot.get_joint.return_value.bounds.return_value = [-2 * math.pi, 2 * math.pi]
        with patch.object(plan, 'execute_and_save'):
            plan.execute_home(arm, robot, {'ra_wrist_3_joint': -3.142}, STAGES[4]['actions'][0])
        target = arm.set_joint_value_target.call_args.args[0]['ra_wrist_3_joint']
        self.assertLess(abs(target - 3.141), 0.001)

    def test_arm_replay_aligns_wrapped_joint_positions_without_mutating_saved_data(self):
        import arm_replay
        arm, robot = Mock(), Mock()
        arm.get_active_joints.return_value = ['ra_wrist_3_joint']
        arm.get_current_joint_values.return_value = [3.141]
        arm.execute.return_value = True
        robot.get_joint.return_value.bounds.return_value = [-2 * math.pi, 2 * math.pi]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['ra_wrist_3_joint']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[-3.142]),
                                              JointTrajectoryPoint(positions=[-3.042])]
        with patch.object(arm_replay.rospy, 'sleep'):
            arm_replay.execute_selected(arm, robot, {'path': trajectory}, 'path', True)
        actual = arm.execute.call_args.args[0]
        self.assertLess(abs(actual.joint_trajectory.points[0].positions[0] - 3.141), 0.001)
        self.assertEqual(trajectory.joint_trajectory.points[0].positions[0], -3.142)
        arm.plan.assert_not_called()

    def test_replay_does_not_shift_an_arm_path_outside_joint_bounds(self):
        import arm_replay
        arm, robot = Mock(), Mock()
        arm.get_active_joints.return_value = ['ra_wrist_3_joint']
        arm.get_current_joint_values.return_value = [3.141]
        robot.get_joint.return_value.bounds.return_value = [-2 * math.pi, 2 * math.pi]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['ra_wrist_3_joint']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[-3.142]),
                                              JointTrajectoryPoint(positions=[6.2])]
        aligned = arm_replay.align_trajectory_to_current(arm, robot, trajectory)
        self.assertEqual(aligned.joint_trajectory.points[0].positions[0], -3.142)

    def test_lift_uses_full_configured_world_z_displacement_with_collision_checks(self):
        from geometry_msgs.msg import PoseStamped
        import rospy
        arm, robot = Mock(), Mock()
        current = PoseStamped()
        current.pose.position.x, current.pose.position.y, current.pose.position.z = 0.3, 0.1, 0.2
        current.pose.orientation.w = 1.0
        arm.get_current_pose.return_value = current
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(2))]
        arm.compute_cartesian_path.return_value = trajectory, 1.0
        arm.retime_trajectory.return_value = trajectory
        with patch.object(plan, 'execute_and_save') as execute, \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': False}):
            plan.execute_cartesian_z_lift(arm, robot, STAGES[3]['actions'][0])
        target = arm.compute_cartesian_path.call_args.args[0][0]
        self.assertAlmostEqual(target.position.z,
                               0.2 + plan.PROJECT_CONFIG['planning']['lift_distance'])
        self.assertEqual(target.position.x, 0.3)
        self.assertEqual(target.position.y, 0.1)
        self.assertIs(arm.compute_cartesian_path.call_args.kwargs['avoid_collisions'], True)
        self.assertEqual(arm.retime_trajectory.call_args.kwargs['velocity_scaling_factor'],
                         plan.PROJECT_CONFIG['planning']['lift_velocity'])
        self.assertEqual(arm.retime_trajectory.call_args.kwargs['acceleration_scaling_factor'],
                         plan.PROJECT_CONFIG['planning']['lift_acceleration'])
        self.assertGreaterEqual(execute.call_args.args[1].joint_trajectory.points[-1].time_from_start.to_sec(),
                                plan.PROJECT_CONFIG['planning']['lift_min_duration'])
        arm.compute_cartesian_path.return_value = trajectory, 0.99
        with patch.object(plan, 'execute_and_save') as execute, \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': False}):
            with self.assertRaisesRegex(RuntimeError, 'lift planning failed'):
                plan.execute_cartesian_z_lift(arm, robot, STAGES[3]['actions'][0])
            execute.assert_not_called()

    def test_rotating_lift_keeps_palm_vertical_and_rejects_partial_alignment(self):
        from geometry_msgs.msg import PoseStamped
        from grasp_pose import palm_pose, rotate
        from graspit_capture import BASE_TO_PALM_ORIENTATION
        import rospy
        arm, robot = Mock(), Mock()
        current = PoseStamped()
        current.pose.position.x, current.pose.position.y, current.pose.position.z = 0.3, 0.1, 0.2
        current.pose.orientation.w = 1
        arm.get_current_pose.return_value = current
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0], time_from_start=rospy.Duration(2))]
        arm.compute_cartesian_path.return_value = trajectory, 1.0
        arm.retime_trajectory.return_value = trajectory
        mount = ([0.149, 0, 0.015], BASE_TO_PALM_ORIENTATION)
        with patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': True, 'lift_min_fraction': 0.8}), \
                patch.object(plan, 'hand_mount', return_value=mount), \
                patch.object(plan, 'execute_and_save') as execute:
            plan.execute_cartesian_z_lift(arm, robot, STAGES[3]['actions'][0])
            waypoints = arm.compute_cartesian_path.call_args.args[0]
            self.assertGreater(len(waypoints), 1)
            for waypoint in waypoints:
                p, q = waypoint.position, waypoint.orientation
                palm = palm_pose(dict(zip(('x','y','z','qx','qy','qz','qw'),
                    [p.x,p.y,p.z,q.x,q.y,q.z,q.w])), *mount)
                self.assertAlmostEqual(palm['translation'][0], 0.449)
                self.assertAlmostEqual(palm['translation'][1], 0.1)
            self.assertAlmostEqual(palm['translation'][2], 0.215+plan.PROJECT_CONFIG['planning']['lift_distance'])
            gravity = rotate(palm['orientation'], plan.PROJECT_CONFIG['grasp']['gravity_direction_palm'])
            self.assertAlmostEqual(gravity[2], -1)
            execute.assert_called_once()
            arm.compute_cartesian_path.return_value = trajectory, 0.99
            execute.reset_mock()
            with self.assertRaisesRegex(RuntimeError, 'lift planning failed'):
                plan.execute_cartesian_z_lift(arm, robot, STAGES[3]['actions'][0])
            execute.assert_not_called()

    def test_rotating_lift_preflight_accounts_for_palm_mount_offset(self):
        from grasp_pose import flange_pose, palm_pose, rotate
        from moveit_msgs.msg import RobotState
        arm, robot = Mock(), Mock()
        arm.get_name.return_value = 'right_arm'
        arm.get_end_effector_link.return_value = 'ra_flange'
        robot.get_current_state.return_value = RobotState()
        grasp = dict(plan.PROJECT_CONFIG['grasp'], translation=[0.4, 0, 0.1],
                     orientation=[0, 0, 0, 1], gravity_direction_palm=[1, 0, 0])
        mount = ([0.149, 0, 0.015], [0, 0, 0, 1])
        solver = Mock(return_value=Mock(error_code=Mock(val=1)))
        with patch.dict(plan.PROJECT_CONFIG, {'grasp': grasp}), \
                patch.dict(plan.PROJECT_CONFIG['planning'], {'lift_align_gravity': True, 'lift_distance': 0.2}), \
                patch.object(plan.rospy, 'wait_for_service'), \
                patch.object(plan.rospy, 'ServiceProxy', return_value=solver):
            plan.verify_lift_endpoint(arm, robot, flange_pose(grasp, *mount))
        goal = solver.call_args.args[0].ik_request.pose_stamped.pose
        self.assertAlmostEqual(goal.position.z, 0.449)
        gravity = rotate([goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w], [1,0,0])
        self.assertAlmostEqual(gravity[2], -1)

    def test_both_modes_invoke_one_shared_replay(self):
        cfg = ctl.configuration()
        with patch.object(ctl, "master"), patch.object(ctl, "ros_command") as execute:
            for mode in ("sim", "real"):
                for command, flag in (("check", "--check"), ("replay", "--step"), ("run", "--run")):
                    ctl.mode_command(cfg, mode, command, [])
                    self.assertEqual(execute.call_args.args[2], ["python3", "/project/app/replay.py", flag])

    def test_arm_pose_execution_failure_stops_planning_workflow(self):
        group = Mock()
        group.execute.return_value = False
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0])]
        with patch.object(plan, "plan", return_value=trajectory), patch.object(plan, "save_traj"):
            with self.assertRaisesRegex(RuntimeError, "execute failed"):
                plan.execute_pose(group, {'x': 0, 'y': 0, 'z': 0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}, STAGES[0]['actions'][0])
        group.stop.assert_called_once()

    def test_missing_master_blocks_nuc_before_ssh(self):
        with patch.object(ctl, "master", side_effect=ctl.UserError("offline")), patch.object(ctl, "ssh") as ssh:
            with self.assertRaises(ctl.UserError):
                ctl.start_nuc(ctl.configuration())
            ssh.assert_not_called()

    def test_failed_calibration_does_not_stop_remote_container(self):
        with patch.object(ctl, "compose", return_value=Mock(stdout="workstation\nur5\n")), \
                patch.object(ctl, "master"), patch.object(ctl, "ros_command", side_effect=subprocess.CalledProcessError(1, "calibration")), patch.object(ctl, "ssh") as ssh:
            with self.assertRaises(subprocess.CalledProcessError):
                ctl.shutdown_real(ctl.configuration())
            ssh.assert_not_called()

    def test_real_shutdown_finishes_when_core_is_gone(self):
        cfg = ctl.configuration()
        with patch.object(ctl, 'compose', return_value=Mock(stdout='workstation\nur5\n')) as compose, \
                patch.object(ctl, 'master', side_effect=ctl.UserError('offline')), \
                patch.object(ctl, 'ros_command') as ros, patch.object(ctl, 'ssh') as ssh:
            ctl.shutdown_real(cfg)
            ros.assert_not_called()
            ssh.assert_called_once()
            compose.assert_called_with(cfg, 'stop', '--timeout', '30', 'ur5', 'workstation')

    def test_real_shutdown_retry_never_contacts_simulation_master(self):
        cfg = ctl.configuration()
        with patch.object(ctl, 'compose', return_value=Mock(stdout='sim\n')) as compose, \
                patch.object(ctl, 'master') as master, patch.object(ctl, 'ros_command') as ros, \
                patch.object(ctl, 'ssh'):
            ctl.shutdown_real(cfg)
            master.assert_not_called()
            ros.assert_not_called()
            compose.assert_called_with(cfg, 'stop', '--timeout', '30', 'ur5', 'workstation')

    def test_modes_cannot_be_started_together(self):
        with patch.object(ctl, "compose", return_value=Mock(stdout="workstation\nur5\n")) as compose:
            with self.assertRaises(ctl.UserError):
                ctl.mode_command(ctl.configuration(), "sim", "up", [])
            self.assertEqual(compose.call_count, 1)


if __name__ == "__main__":
    unittest.main()
