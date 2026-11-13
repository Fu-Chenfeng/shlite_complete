"""Capture selection and parsing checks using the actual exported grasp."""
import copy
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from graspit_capture import resolve_capture
from grasp_pose import rotate
from project_config import load_config
from task_definition import save_task, load_task


class CaptureTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        shutil.copytree(ROOT / 'assets', self.root / 'assets')
        shutil.copy(ROOT / 'assets/grasps/261110/261110.txt', self.root / '261110.txt')
        self.config = yaml.safe_load((ROOT / 'config.yaml').read_text())
        self.config['grasp'].update(capture_file='261110.txt', capture_index=0)
        self.config['planning']['lift_align_gravity'] = False

    def load(self):
        (self.root / 'config.yaml').write_text(yaml.safe_dump(self.config))
        return load_config(self.root)

    def test_selected_root_file_reproduces_import_and_generates_targets(self):
        resolved = self.load()['grasp']
        for actual, expected in zip(resolved['translation'], self.config['grasp']['translation']):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(resolved['joint_angles']['rh_FFJ2'], 0.1875)
        self.assertAlmostEqual(resolved['joint_angles']['rh_FFJ1'], 0.15)
        self.assertAlmostEqual(resolved['joint_angles']['rh_THJ2'], math.radians(40))
        self.assertEqual(len(resolved['capture_sha256']), 64)
        # Capture mode does not require the manual fallback fields.
        for field in ('translation', 'orientation', 'joint_angles', 'tightened_joint_angles'):
            del self.config['grasp'][field]
        self.assertEqual(self.load()['grasp']['joint_angles'], resolved['joint_angles'])

    def test_changing_filename_and_penultimate_joint_increment_changes_resolved_targets(self):
        original = (self.root / '261110.txt').read_text()
        (self.root / 'alternate.txt').write_text(original.replace('0.325817', '0.100000', 1))
        self.config['grasp'].update(capture_file='alternate.txt', capture_tighten_degrees=5)
        resolved = self.load()['grasp']
        self.assertEqual(resolved['joint_angles']['rh_FFJ4'], 0.1)
        for name, angle in resolved['joint_angles'].items():
            expected = math.radians(5) if name in {'rh_FFJ2', 'rh_MFJ2', 'rh_RFJ2'} else 0
            self.assertAlmostEqual(resolved['tightened_joint_angles'][name]-angle, expected)
        self.assertAlmostEqual(resolved['tightened_joint_angles']['rh_THJ2'], math.radians(40))
        self.assertEqual(resolved['tightening_limited_joints']['rh_THJ2']['requested_degrees'], 5)
        self.assertAlmostEqual(resolved['tightening_limited_joints']['rh_THJ2']['applied_degrees'], 0)

    def test_added_tightening_is_capped_at_both_limits_without_changing_capture(self):
        original = self.load()['grasp']['joint_angles']
        for degrees, finger_limit, thumb_limit in ((200, 90, 40), (-200, 0, -40)):
            self.config['grasp']['capture_tighten_degrees'] = degrees
            resolved = self.load()['grasp']
            self.assertEqual(resolved['joint_angles'], original)
            for name, value in original.items():
                expected = math.radians(thumb_limit if name == 'rh_THJ2' else finger_limit) if name.endswith('J2') else value
                self.assertAlmostEqual(resolved['tightened_joint_angles'][name], expected)
            self.assertEqual(set(resolved['tightening_limited_joints']), {'rh_FFJ2', 'rh_MFJ2', 'rh_RFJ2', 'rh_THJ2'})

    def test_object_translation_and_rotation_relocate_grasp(self):
        original = self.load()['grasp']
        object_position = self.config['object']['pose'][:3]
        self.config['object']['pose'][5] += math.pi/2
        self.config['object']['pose'][0] += 0.1
        updated = self.load()['grasp']
        local = [a-b for a, b in zip(original['translation'], object_position)]
        rotated = rotate([0, 0, math.sin(math.pi/4), math.cos(math.pi/4)], local)
        for actual, relative, offset in zip(updated['translation'], rotated, self.config['object']['pose'][:3]):
            self.assertAlmostEqual(actual, relative+offset)

    def test_manual_mode_ignores_capture_and_preserves_explicit_targets(self):
        self.config['grasp'].update(mode='manual', capture_file='missing.txt')
        self.assertEqual(self.load()['grasp'], self.config['grasp'])
        del self.config['grasp']['mode']  # Older configs still use manual values.
        self.assertEqual(self.load()['grasp'], self.config['grasp'])

    def test_missing_outside_and_malformed_sources_fail_as_config_errors(self):
        for filename in ('missing.txt', '../outside.txt'):
            self.config['grasp']['capture_file'] = filename
            with self.assertRaisesRegex(ValueError, 'Invalid config.yaml'):
                self.load()
        self.config['grasp']['capture_file'] = '261110.txt'
        original = (self.root / '261110.txt').read_text()
        for invalid in ('', '5 1 2\n0 1 2', original.replace('0.325817', 'nan', 1),
                        original.replace('0.325817', '2.000000', 1),
                        original.replace('-0.730528 -0.634329 0.169088 0.188055', '0 0 0 0', 1)):
            (self.root / '261110.txt').write_text(invalid)
            with self.assertRaisesRegex(ValueError, 'Invalid config.yaml'):
                self.load()

    def test_saved_task_keeps_resolved_pose_without_source_file(self):
        config = self.load()
        save_task(self.root, config)
        (self.root / '261110.txt').unlink()
        saved = load_task(self.root)
        self.assertEqual(saved['grasp'], config['grasp'])

    def test_invalid_mode_or_tightening_setting_is_rejected(self):
        for change in ({'mode': 'typo'}, {'capture_tighten_degrees': float('nan')},
                       {'capture_file': ''}):
            self.config['grasp'] = copy.deepcopy(yaml.safe_load((ROOT / 'config.yaml').read_text())['grasp'])
            self.config['grasp'].update(change)
            with self.assertRaisesRegex(ValueError, 'Invalid config.yaml'):
                self.load()


if __name__ == '__main__':
    unittest.main()
