import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from joint_targets import thumb_base_target, approach_hand_target
from project_config import load_config
from task_definition import THUMB_PREPARATION_JOINTS, save_task, load_task, thumb_preparation_joints


class ThumbPreparationTest(unittest.TestCase):
    def test_approach_fully_prepares_bases_and_blends_remaining_joints_from_live_angles(self):
        grasp = load_config()['grasp']['joint_angles']
        robot, hand = Mock(), Mock()
        hand.get_active_joints.return_value = list(grasp)
        hand.get_current_joint_values.return_value = [0.1] * len(grasp)
        robot.get_joint.return_value.bounds.return_value = [-2, 2]
        for fraction in (0, 0.6, 1):
            result = approach_hand_target(robot, hand, grasp, fraction)
            for name, goal in grasp.items():
                expected = goal if name in THUMB_PREPARATION_JOINTS or name.endswith('J4') else 0.1 + fraction * (goal - 0.1)
                self.assertAlmostEqual(result[name], expected)
        for fraction in (-0.1, 1.1, float('nan'), True):
            with self.assertRaises(ValueError):
                approach_hand_target(robot, hand, grasp, fraction)

    def test_both_base_joints_prepare_while_other_angles_hold(self):
        config = load_config()
        robot, hand = Mock(), Mock()
        names = list(config['grasp']['joint_angles'])
        hand.get_active_joints.return_value = names
        hand.get_current_joint_values.return_value = [0.1]*len(names)
        robot.get_joint.return_value.bounds.return_value = [-2, 2]
        requested = {name: config['grasp']['joint_angles'][name] for name in THUMB_PREPARATION_JOINTS}
        target = thumb_base_target(robot, hand, requested)
        for name, value in target.items():
            self.assertEqual(value, requested.get(name, 0.1))
        # Legacy recordings keep their original one-joint preparation.
        old = thumb_base_target(robot, hand, {'rh_THJ5': requested['rh_THJ5']})
        self.assertEqual(old['rh_THJ4'], 0.1)

    def test_recording_declares_preparation_joints_and_old_runs_keep_one(self):
        with tempfile.TemporaryDirectory() as folder:
            save_task(folder, load_config())
            task = load_task(folder)
            self.assertEqual(task['hand_preparation_mode'], 'saved_target')
            self.assertEqual(thumb_preparation_joints(task), ['rh_THJ5', 'rh_THJ4'])
            del task['thumb_preparation_joints']
            (Path(folder) / 'task.json').write_text(json.dumps(task))
            self.assertEqual(thumb_preparation_joints(load_task(folder)), ['rh_THJ5'])


if __name__ == '__main__':
    unittest.main()
