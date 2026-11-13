"""World Z offset moves only the target object and task grasp."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from project_config import load_config, object_pose, grasp_target, prepare_simulation, launch_arguments
from task_definition import save_task, load_task


class SceneOffsetTest(unittest.TestCase):
    def test_initial_z_changes_only_robot_placement_in_both_modes(self):
        config = load_config()
        original = copy.deepcopy(config)
        config['robot']['initial_z'] = 0.032
        for mode in ('sim', 'real'):
            arguments = launch_arguments(config, mode)
            self.assertIn('initial_z:=0.032', arguments)
        self.assertEqual(object_pose(config), object_pose(original))
        self.assertEqual(grasp_target(config), grasp_target(original))
        self.assertEqual(config['environment'], original['environment'])
        self.assertEqual(config['robot']['home_joints'], original['robot']['home_joints'])

    def test_recording_snapshots_shifted_grasp_and_object_pose(self):
        config = load_config()
        base_height = config['grasp']['translation'][2]
        config['object']['grasp_z_offset'] = 0.02
        config['grasp'] = grasp_target(config)
        with tempfile.TemporaryDirectory() as folder:
            save_task(folder, config)
            task = load_task(folder)
        self.assertAlmostEqual(task['grasp']['translation'][2], base_height + 0.02)
        self.assertAlmostEqual(task['scene']['object_pose'][2], config['object']['pose'][2] + 0.02)
        self.assertEqual(task['scene']['object_grasp_z_offset'], 0.02)

    def test_offset_moves_object_and_both_grasp_sources_without_mutating_config(self):
        config = load_config()
        for mode in ('manual', 'graspit'):
            config['grasp']['mode'] = mode
            for offset in (0, 0.02, -0.01):
                config['object']['grasp_z_offset'] = offset
                original = copy.deepcopy(config)
                obj, hand = object_pose(config), grasp_target(config)
                self.assertEqual(obj[:2] + obj[3:], config['object']['pose'][:2] + config['object']['pose'][3:])
                self.assertAlmostEqual(obj[2], config['object']['pose'][2] + offset)
                self.assertAlmostEqual(hand['translation'][2], config['grasp']['translation'][2] + offset)
                self.assertEqual(hand['translation'][:2], config['grasp']['translation'][:2])
                self.assertEqual(hand['orientation'], config['grasp']['orientation'])
                self.assertEqual(hand['joint_angles'], config['grasp']['joint_angles'])
                self.assertEqual(config, original)

    def test_generated_world_keeps_floor_fixed_when_object_moves(self):
        import shutil
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            shutil.copytree(ROOT / 'assets', root / 'assets')
            config = load_config()
            config['object']['grasp_z_offset'] = 0
            baseline = ET.parse(prepare_simulation(config, root=root))
            config['object']['grasp_z_offset'] = 0.02
            raised = ET.parse(prepare_simulation(config, root=root))
            def model(tree, name):
                return tree.find("world/model[@name='%s']" % name)
            self.assertEqual(ET.tostring(model(baseline, 'ground_plane_user')),
                             ET.tostring(model(raised, 'ground_plane_user')))
            before = list(map(float, model(baseline, 'rect_object').findtext('pose').split()))
            after = list(map(float, model(raised, 'rect_object').findtext('pose').split()))
            self.assertAlmostEqual(after[2]-before[2], 0.02)
            self.assertEqual(after[:2]+after[3:], before[:2]+before[3:])


if __name__ == '__main__':
    unittest.main()
