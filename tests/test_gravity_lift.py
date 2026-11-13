import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from grasp_pose import (aligned_orientation, conjugate, flange_pose, lift_endpoint,
                        lift_waypoints, multiply, palm_pose, rotate, unit)
from graspit_capture import BASE_TO_PALM_ORIENTATION, resolve_capture
from project_config import load_config, grasp_target
from task_definition import save_task, load_task


class GravityLiftTest(unittest.TestCase):
    def assertVector(self, actual, expected, places=8):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=places)

    def test_json_reads_actual_joint_values_without_dh_offsets(self):
        grasp = load_config()['grasp']
        self.assertAlmostEqual(grasp['joint_angles']['rh_FFJ4'], math.radians(20))
        self.assertAlmostEqual(grasp['joint_angles']['rh_FFJ3'], math.pi/2)
        self.assertAlmostEqual(grasp['joint_angles']['rh_RFJ3'], 1.2093963263138718)
        self.assertAlmostEqual(grasp['joint_angles']['rh_FFJ1'], 0.0625)
        bend = math.radians(grasp['capture_tighten_degrees'])
        for name, angle in grasp['joint_angles'].items():
            expected = bend if name in {'rh_FFJ2', 'rh_MFJ2', 'rh_RFJ2', 'rh_THJ2'} else 0.0
            self.assertAlmostEqual(grasp['tightened_joint_angles'][name] - angle, expected)
        self.assertVector(rotate(grasp['orientation'], grasp['gravity_direction_palm']),
                          [0.7122151531290275, 0.4429733372030698, -0.5445403549605587])
        self.assertVector(rotate(aligned_orientation(grasp['orientation'], grasp['gravity_direction_palm']),
                                 grasp['gravity_direction_palm']), [0, 0, -1])

    def test_capture_world_transform_does_not_change_object_relative_grasp(self):
        document = json.loads((ROOT / 'grasps.json').read_text())
        config = load_config()
        original = config['grasp']
        q = unit([0.2, -0.3, 0.4, 0.8])
        record = document['grasps'][0]
        for name in ('hand_pose_world', 'object_pose_world'):
            pose = record[name]
            pose['translation_mm'] = [v+d for v, d in zip(rotate(q, pose['translation_mm']), [100, -50, 70])]
            pose['quaternion_xyzw'] = multiply(q, pose['quaternion_xyzw'])
        record['gravity_direction_world'] = rotate(q, record['gravity_direction_world'])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'moved.json'
            path.write_text(json.dumps(document))
            result = resolve_capture(original, config['object']['pose'], path)
        self.assertVector(result['translation'], original['translation'])
        self.assertVector(result['orientation'], original['orientation'])
        self.assertVector(result['gravity_direction_palm'], original['gravity_direction_palm'])

    def test_parallel_and_opposite_gravity_alignment_is_finite(self):
        for gravity in ([0, 0, -1], [0, 0, 1], [1, 0, 0]):
            q = aligned_orientation([0, 0, 0, 1], gravity)
            self.assertVector(rotate(q, gravity), [0, 0, -1])
            self.assertAlmostEqual(sum(v*v for v in q), 1)
        with self.assertRaises(ValueError):
            aligned_orientation([0, 0, 0, 1], [0, 0, 0])

    def test_palm_rises_vertically_while_flange_compensates_for_rotation(self):
        start = grasp_target(load_config())
        mount_t, mount_q = [0.149, 0, 0.015], BASE_TO_PALM_ORIENTATION
        palms = lift_waypoints(start, 0.2, 0.005, start['gravity_direction_palm'], math.radians(2))
        previous_z = start['translation'][2]
        previous_q = start['orientation']
        for palm in palms:
            actual = palm_pose(flange_pose(palm, mount_t, mount_q), mount_t, mount_q)
            self.assertVector(actual['translation'][:2], start['translation'][:2])
            self.assertGreater(actual['translation'][2], previous_z)
            self.assertLessEqual(actual['translation'][2]-previous_z, 0.005+1e-10)
            angle = 2*math.acos(min(1, abs(sum(x*y for x, y in zip(previous_q, palm['orientation'])))))
            self.assertLessEqual(angle, math.radians(2)+1e-8)
            previous_z, previous_q = actual['translation'][2], palm['orientation']
        self.assertAlmostEqual(previous_z-start['translation'][2], 0.2)
        self.assertVector(rotate(palms[-1]['orientation'], start['gravity_direction_palm']), [0, 0, -1])
        initial_flange = flange_pose(start, mount_t, mount_q)
        endpoint = lift_endpoint(initial_flange, start,
                                 {'lift_distance': 0.2, 'lift_align_gravity': True})
        self.assertVector(palm_pose(endpoint, mount_t, mount_q)['translation'], palms[-1]['translation'])

    def test_capture_index_selects_grasp_and_rejects_invalid_records(self):
        config = load_config()
        document = json.loads((ROOT / 'grasps.json').read_text())
        second = copy.deepcopy(document['grasps'][0])
        second['gravity_direction_world'] = [0, -1, 0]
        document['grasps'].append(second)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'captures.json'
            path.write_text(json.dumps(document))
            selected = resolve_capture(dict(config['grasp'], capture_index=1), config['object']['pose'], path)
            self.assertVector(selected['gravity_direction_graspit_world'], [0, -1, 0])
            for index in (-1, 2, True):
                with self.assertRaises(ValueError):
                    resolve_capture(dict(config['grasp'], capture_index=index), config['object']['pose'], path)
            for change in ('zero_gravity', 'missing_object', 'duplicate_joint'):
                bad = copy.deepcopy(document)
                if change == 'zero_gravity':
                    bad['grasps'][0]['gravity_direction_world'] = [0, 0, 0]
                elif change == 'missing_object':
                    del bad['grasps'][0]['object_pose_world']
                else:
                    bad['grasps'][0]['joints'].append(bad['grasps'][0]['joints'][0])
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    resolve_capture(config['grasp'], config['object']['pose'], path)

    def test_alignment_metadata_is_saved_for_replay(self):
        config = load_config()
        config['grasp'] = grasp_target(config)
        with tempfile.TemporaryDirectory() as folder:
            save_task(folder, config)
            task = load_task(folder)
        self.assertTrue(task['planning']['lift_align_gravity'])
        alignment = task['lift_alignment']
        self.assertVector(rotate(alignment['target_palm_orientation'], alignment['gravity_direction_palm']),
                          [0, 0, -1])


if __name__ == '__main__':
    unittest.main()
