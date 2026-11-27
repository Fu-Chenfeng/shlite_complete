"""MoveIt custom planning continuity, capture and saved-file checks; no motion."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import custom_motion as motion
from custom_trajectory import Sequence, TASK_VERSION
from trajectory_store import create_run, finish_run, replay_run_choices


class CustomMotionTest(unittest.TestCase):
    def setUp(self):
        self.robot = Mock()
        self.robot.get_joint.return_value.bounds.return_value = [-2, 2]
        self.state = RobotState()
        self.state.joint_state.name = ['a', 'h']
        self.state.joint_state.position = [0, 0]
        self.robot.get_current_state.return_value = self.state
        self.groups = {'arm': Mock(), 'hand': Mock()}
        for backend, name in (('arm', 'a'), ('hand', 'h')):
            self.groups[backend].get_active_joints.return_value = [name]
            self.groups[backend].get_current_joint_values.return_value = [0]
        self.sequence = Sequence({'arm': {'a': 0}, 'hand': {'h': 0}})
        self.sequence.add().update(arm={'a': 1}, hand={'h': .5}, order='hand_arm')
        self.sequence.add().update(arm={'a': .2}, hand={'h': .1})
        self.document = self.sequence.document('custom_test')
        self.addCleanup(patch.stopall)
        patch.object(motion.rospy, 'is_shutdown', return_value=False).start()
        patch.object(motion.rospy, 'loginfo').start()

    def trajectory(self, name, start, end):
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = [name]
        traj.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[start], time_from_start=rospy.Duration(0)),
            JointTrajectoryPoint(positions=[end], time_from_start=rospy.Duration(1))]
        return traj

    def plans(self):
        self.groups['arm'].plan.side_effect = [(True, self.trajectory('a', 0, 1), 0, None),
                                              (True, self.trajectory('a', 1, .2), 0, None)]
        self.groups['hand'].plan.side_effect = [(True, self.trajectory('h', 0, .5), 0, None),
                                               (True, self.trajectory('h', .5, .1), 0, None)]
        return motion.plan_sequence(self.robot, self.groups, self.document)

    def test_plans_all_tasks_from_previous_endpoint_in_requested_order(self):
        planned = self.plans()
        self.assertEqual([a['group'] for a, _, _ in planned], ['hand', 'arm', 'arm', 'hand'])
        arm_starts = self.groups['arm'].set_start_state.call_args_list
        self.assertEqual(list(arm_starts[0].args[0].joint_state.position), [0, .5])
        self.assertEqual(list(arm_starts[1].args[0].joint_state.position), [1, .5])
        self.assertEqual(list(self.state.joint_state.position), [0, 0])
        for group in self.groups.values():
            group.execute.assert_not_called()

    def test_later_planning_failure_never_executes_partial_sequence(self):
        self.groups['hand'].plan.return_value = (True, self.trajectory('h', 0, .5), 0, None)
        self.groups['arm'].plan.return_value = (False, self.trajectory('a', 0, 1), 0, None)
        with self.assertRaisesRegex(RuntimeError, 'no motion started'):
            motion.plan_sequence(self.robot, self.groups, self.document)
        for group in self.groups.values():
            group.execute.assert_not_called()

    def test_all_targets_validated_before_any_planning(self):
        self.document['tasks'][1]['hand']['h'] = 3
        with self.assertRaises(ValueError):
            motion.plan_sequence(self.robot, self.groups, self.document)
        for group in self.groups.values():
            group.plan.assert_not_called()

    def test_capture_uses_editable_rviz_state_instead_of_live_state(self):
        goal = copy.deepcopy(self.state)
        goal.joint_state.position = [1.1, .4]
        with patch.object(motion, 'require_simulation'), patch.object(motion.rospy, 'wait_for_message', return_value=goal):
            result = motion.capture(self.robot, self.groups, 'rviz')
        self.assertEqual(result['arm'], {'a': 1.1})
        self.assertEqual(result['hand'], {'h': .4})
        self.robot.get_current_state.assert_not_called()

    def test_execution_rejects_changed_start_and_stops_on_failure(self):
        action = {'group': 'arm', 'file': 'test.pkl'}
        planned = [(action, self.trajectory('a', 1, .2), {'a': .2})]
        with self.assertRaisesRegex(RuntimeError, 'Robot moved'):
            motion.execute_plans(self.groups, planned, .01)
        self.groups['arm'].execute.assert_not_called()
        self.groups['arm'].get_current_joint_values.return_value = [1]
        self.groups['arm'].execute.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'Execution failed'):
            motion.execute_plans(self.groups, planned, .01)
        self.groups['arm'].stop.assert_called_once()

    def test_rviz_capture_normalizes_only_small_feedback_drift_past_stops(self):
        goal = copy.deepcopy(self.state)
        goal.joint_state.position = [1, 2.00001]
        with patch.object(motion, 'require_simulation'), patch.object(motion.rospy, 'wait_for_message', return_value=goal):
            self.assertEqual(motion.capture(self.robot, self.groups, 'rviz')['hand'], {'h': 2})
            goal.joint_state.position = [1, 2.1]
            with self.assertRaisesRegex(ValueError, 'outside limits'):
                motion.capture(self.robot, self.groups, 'rviz')

    def test_save_check_and_archive_round_trip(self):
        planned = self.plans()
        with tempfile.TemporaryDirectory() as folder:
            run = create_run(folder, task_version=TASK_VERSION, name='custom_test')
            motion.save_sequence(run, self.document, {'planning': {'joint_tolerance': .01}}, planned)
            finish_run(run, True)
            task, loaded = motion.load_recording(run, self.robot, self.groups)
            self.assertEqual(task['tasks'], self.document['tasks'])
            self.assertEqual(len(loaded), 4)
            self.assertTrue(replay_run_choices(folder)[0]['replayable'])
            for group in self.groups.values():
                group.execute.assert_not_called()

    def test_custom_replay_prompts_per_task_and_preserves_action_order(self):
        import arm_replay
        import hand_replay
        import replay
        import sim_contacts
        planned = self.plans()
        task = {**self.document, 'stages': motion.stages(self.document)}
        events = []
        restore = Mock()
        with patch.object(motion, 'load_recording', return_value=(task, planned)), \
                patch.object(sim_contacts, 'disable_replay_collision_checks', return_value=restore), \
                patch.object(replay, 'is_simulation_replay', return_value=True), \
                patch.object(replay, 'simulation_replay_speed', return_value=1), \
                patch.object(arm_replay, 'execute_selected', side_effect=lambda *a, **kw: events.append('arm')), \
                patch.object(hand_replay, 'plan_and_execute_target', side_effect=lambda *a, **kw: events.append('hand')), \
                patch('builtins.input', side_effect=['', 'q']) as prompt:
            motion.replay(Path('custom_test'), self.robot, self.groups, step=True, execute=True)
        self.assertEqual(events, ['hand', 'arm'])
        self.assertEqual(prompt.call_count, 2)
        restore.assert_called_once()
        for group in self.groups.values():
            group.stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
