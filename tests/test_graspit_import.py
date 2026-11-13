"""Check captured joint coupling, common frames and mesh geometry without ROS."""
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from grasp_pose import rotate
from project_config import load_config, object_model
from simulation_models import sync_models
from graspit_capture import resolve_capture


class GraspItImportTest(unittest.TestCase):
    def legacy_grasp(self):
        config = load_config()
        grasp = dict(config['grasp'], capture_index=0)
        return resolve_capture(grasp, config['object']['pose'], ROOT / 'assets/grasps/261110/261110.txt')
    def test_bridge_model_uses_current_mesh_without_double_world_offset(self):
        config = load_config()
        config['object']['collision_mesh'] = True
        model = object_model(config)
        ET.SubElement(model, 'pose').text = '0.41 0 0.034472 1.570796 0 0'
        sdf = ET.Element('sdf', version='1.6')
        ET.SubElement(sdf, 'world', name='test').append(model)
        with tempfile.TemporaryDirectory() as folder:
            world = Path(folder) / 'world.sdf'
            ET.ElementTree(sdf).write(world)
            sync_models(world, Path(folder) / 'models')
            actual = ET.parse(Path(folder) / 'models/rect_object/model.sdf')
            self.assertIsNone(actual.find('model/pose'))
            self.assertEqual(actual.findtext('model/link/collision/geometry/mesh/uri'),
                             '/project/assets/models/tofuprintfinal/tofuprintfinal.stl')
            self.assertEqual(actual.findtext('model/link/collision/geometry/mesh/scale'),
                             '0.001 0.001 0.001')

    def test_capture_coupling_is_preserved(self):
        raw = (ROOT / 'assets/grasps/261110/261110.txt').read_text().splitlines()
        dofs = list(map(float, raw[0].split()[1:]))
        joints = self.legacy_grasp()['joint_angles']
        for chain, finger in enumerate(('FF', 'MF', 'RF')):
            for joint, value in zip((4, 3, 2, 1),
                                    (dofs[3*chain], dofs[3*chain+1],
                                     dofs[3*chain+2], 0.8*dofs[3*chain+2])):
                self.assertAlmostEqual(joints['rh_%sJ%d' % (finger, joint)], value, places=6)
        for joint, value in zip((5, 4, 2, 1), dofs[9:]):
            self.assertAlmostEqual(joints['rh_THJ%d' % joint], value, places=6)

    def test_actual_palm_pose_matches_all_captured_joint_origins(self):
        report = json.loads((ROOT / 'assets/grasps/261110/alignment.json').read_text())
        grasp = self.legacy_grasp()
        rebase = report['graspit_to_ros_world']
        for local, captured in zip(report['fit']['palm_points'], report['fit']['captured_points']):
            actual = [a+b for a, b in zip(rotate(grasp['orientation'], local), grasp['translation'])]
            expected = [a+b for a, b in zip(rotate(rebase['rotation_xyzw'], captured), rebase['translation_m'])]
            self.assertLess(math.dist(actual, expected), 1e-6)

    def test_visual_and_collision_mesh_match_and_rest_on_floor(self):
        config = load_config()
        config['object']['collision_mesh'] = True
        model = object_model(config)
        self.assertEqual(model.findtext('link/visual/geometry/mesh/uri'),
                         model.findtext('link/collision/geometry/mesh/uri'))
        data = (ROOT / config['object']['mesh']).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        self.assertEqual(count, 3134)
        self.assertEqual(len(data), 84 + 50 * count)
        vertices = [struct.unpack_from('<12fH', data, 84+50*i)[3:12] for i in range(count)]
        # +90 degrees around X maps the mesh's original Y onto world Z.
        bottom = min(face[j] for face in vertices for j in (1, 4, 7)) * 0.001
        self.assertAlmostEqual(bottom + config['object']['pose'][2], 0, places=7)


if __name__ == '__main__':
    unittest.main()
