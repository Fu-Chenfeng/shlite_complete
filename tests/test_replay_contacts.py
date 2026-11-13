"""Replay collision overrides must be scoped to execution, including failures."""
from contextlib import ExitStack
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import replay
import arm_replay


class ReplayContactsTest(unittest.TestCase):
    def test_real_arm_path_collision_between_saved_points_blocks_motion(self):
        from moveit_msgs.msg import RobotState, RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        state = RobotState()
        state.joint_state.name = ['ra_joint', 'rh_joint']
        state.joint_state.position = [0.0, 0.4]
        robot, arm = Mock(), Mock()
        robot.get_current_state.return_value = state
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['ra_joint']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0]),
                                              JointTrajectoryPoint(positions=[0.1])]
        def check(robot_state, group_name):
            self.assertEqual(group_name, '')
            self.assertEqual(robot_state.joint_state.position[1], 0.4)
            angle = robot_state.joint_state.position[0]
            return Mock(valid=not (0.03 < angle < 0.07),
                        contacts=[Mock(contact_body_1='ra_forearm_link', contact_body_2='ground_plane_user')])
        with patch.object(arm_replay, 'align_trajectory_to_current', return_value=trajectory), \
                patch.object(arm_replay.rospy, 'wait_for_service'), \
                patch.object(arm_replay.rospy, 'is_shutdown', return_value=False), \
                patch.object(arm_replay.rospy, 'ServiceProxy', return_value=check), \
                patch.object(arm_replay, 'plan_and_execute_to_start') as move_start:
            with self.assertRaisesRegex(RuntimeError, 'Real replay path blocked.*ground_plane_user'):
                arm_replay.execute_selected(arm, robot, {'lift': trajectory}, 'lift', True, check_collisions=True)
        move_start.assert_not_called()
        arm.execute.assert_not_called()

    def test_override_restored_on_completion_failure_and_not_applied_for_check(self):
        task = {'planning': {'lift_distance': .2, 'joint_tolerance': .01}}
        for check_only, failed in ((False, False), (False, True), (True, False)):
            with self.subTest(check_only=check_only, failed=failed), ExitStack() as stack:
                for name in ('init_node', 'on_shutdown'):
                    stack.enter_context(patch.object(replay.rospy, name))
                stack.enter_context(patch.object(replay.rospy, 'myargv', return_value=['replay', '--check' if check_only else '--run']))
                for name in ('roscpp_initialize', 'roscpp_shutdown', 'RobotCommander', 'MoveGroupCommander'):
                    stack.enter_context(patch.object(replay.moveit_commander, name))
                stack.enter_context(patch.object(replay, 'configured_run', return_value=Path('/recording')))
                stack.enter_context(patch.object(replay, 'load_task', return_value=task))
                stack.enter_context(patch.object(replay, 'load_recording', return_value=(task, {}, {})))
                stack.enter_context(patch.object(replay, 'thumb_preparation_joints', return_value=('rh_THJ5',)))
                stack.enter_context(patch.object(replay, 'simulation_replay_speed', return_value=3.0))
                stack.enter_context(patch.object(replay, 'is_simulation_replay', return_value=False))
                restored = Mock()
                disable = stack.enter_context(patch.object(replay, 'disable_replay_collision_checks', return_value=restored))
                def execute(*args, **kwargs):
                    disable.assert_called_once()
                    self.assertTrue(disable.call_args.kwargs['protect_table'])
                    self.assertTrue(kwargs['check_table'])
                    restored.assert_not_called()
                    if failed:
                        raise RuntimeError('execution failed')
                run = stack.enter_context(patch.object(replay, 'execute_stages', side_effect=execute))
                if failed:
                    with self.assertRaisesRegex(RuntimeError, 'execution failed'):
                        replay.main()
                else:
                    replay.main()
                if check_only:
                    disable.assert_not_called()
                    restored.assert_not_called()
                    run.assert_not_called()
                else:
                    restored.assert_called_once_with()
