"""Custom task inheritance, recording isolation and archive compatibility."""
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
from custom_trajectory import Sequence, TASK_VERSION, default_name, load_task, stages, validate_sequence, configure_rviz
from desktop_session import Session
from task_definition import TASK_VERSION as GRASP_VERSION
from trajectory_store import create_run, finish_run, replay_run_choices, resolve_run


def state():
    return {'source': 'rviz', 'arm': {'arm_joint': .1}, 'hand': {'hand_joint': .2},
            'bounds': {'arm_joint': [-2, 2], 'hand_joint': [-1, 1]}}


class CustomSequenceTest(unittest.TestCase):
    def test_new_tasks_inherit_previous_angles_without_aliasing(self):
        seq = Sequence(state())
        first = seq.add()
        self.assertEqual(first['hand'], {'hand_joint': .2})
        first['arm']['arm_joint'] = 1.2
        second = seq.add()
        self.assertEqual(second['arm'], first['arm'])
        second['hand']['hand_joint'] = .5
        self.assertEqual(first['hand']['hand_joint'], .2)
        document = seq.document('custom_test')
        document['tasks'][0]['arm']['arm_joint'] = 0
        self.assertEqual(first['arm']['arm_joint'], 1.2)

    def test_recording_is_a_full_independent_snapshot(self):
        seq = Sequence(state())
        captured = state()
        recorded = seq.capture(captured)
        captured['hand']['hand_joint'] = 1
        self.assertEqual(recorded['hand']['hand_joint'], .2)
        self.assertEqual(recorded['name'], 'State 1')
        self.assertTrue(recorded['captured_at'].endswith('+09:00'))

    def test_default_name_uses_tokyo_time(self):
        self.assertEqual(default_name(datetime(2026, 11, 27, 23, 1, 2, tzinfo=timezone.utc)),
                         'custom_2026-11-28_08-01-02')

    def test_bad_sequence_rejected_before_planning(self):
        seq = Sequence(state())
        with self.assertRaises(ValueError):
            seq.document('custom_empty')
        seq.add()
        valid = seq.document('custom_test')
        for change in (lambda s: s.update(name='../escape'), lambda s: s.update(name='latest'),
                       lambda s: s['tasks'][0].update(order='simultaneous'),
                       lambda s: s['tasks'][0]['hand'].update(hand_joint=float('nan')),
                       lambda s: s['tasks'][0]['hand'].update(hand_joint=True),
                       lambda s: s['tasks'][0].update(arm={})):
            broken = copy.deepcopy(valid)
            change(broken)
            with self.assertRaises(ValueError):
                validate_sequence(broken)
        seq.add()['hand']['extra_joint'] = 0
        with self.assertRaises(ValueError):
            seq.document('different_joints')

    def test_custom_runs_are_listed_and_selected_alongside_grasp_runs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            seq = Sequence(state())
            seq.add()['order'] = 'hand_arm'
            doc = seq.document('custom_test')
            doc.update(stages=stages(doc), planning={'joint_tolerance': .01})
            run = create_run(root, task_version=TASK_VERSION, name=doc['name'])
            (run / 'task.json').write_text(json.dumps(doc))
            for stage in doc['stages']:
                for action in stage['actions']:
                    (run / action['group'] / action['file']).touch()
                    if 'target' in action:
                        (run / 'hand' / action['target']).touch()
            finish_run(run, True)
            self.assertEqual(load_task(run), doc)
            self.assertEqual(replay_run_choices(root), [{'name': run.name, 'status': 'ready', 'replayable': True}])
            self.assertEqual(resolve_run(root, task_version=(GRASP_VERSION, TASK_VERSION)), run)
            repeated = create_run(root, task_version=TASK_VERSION, name=doc['name'])
            self.assertEqual(repeated.name, 'custom_test_001')
            doc['stages'][0]['actions'][0]['file'] = '../escape.pkl'
            (run / 'task.json').write_text(json.dumps(doc))
            with self.assertRaises(ValueError):
                load_task(run)
            self.assertFalse(replay_run_choices(root)[1]['replayable'])

    def test_rviz_bridge_preserves_existing_displays_and_settings(self):
        import yaml
        config = {'Visualization Manager': {'Displays': [
            {'Class': 'rviz/Grid', 'Name': 'Grid'},
            {'Class': 'moveit_rviz_plugin/MotionPlanning', 'Name': 'MotionPlanning', 'Planning Group': 'right_arm'}]}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'moveit.rviz'
            path.write_text(yaml.safe_dump(config))
            configure_rviz(path)
            configure_rviz(path)
            displays = yaml.safe_load(path.read_text())['Visualization Manager']['Displays']
            self.assertEqual(displays[0], config['Visualization Manager']['Displays'][0])
            self.assertEqual(displays[1]['Planning Group'], 'right_arm')
            self.assertEqual(displays[1]['Class'], 'shlite_custom_rviz/MotionPlanning')

    def test_capture_reads_json_despite_ros_console_prefix(self):
        session = Session('sim', Mock())
        session.ready = True
        with patch.object(session, 'run', return_value='[INFO] ...CUSTOM_STATE_JSON=' + json.dumps(state()) + '\n'):
            self.assertEqual(session.capture_state(), state())
        session.ready = False
        with self.assertRaises(RuntimeError):
            session.capture_state()


if __name__ == '__main__':
    unittest.main()
