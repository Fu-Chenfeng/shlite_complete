"""Check real scene selection without starting ROS or sending robot commands."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
import add_cell_obstacles as obstacles
from project_config import load_config, object_pose


class RealSceneTest(unittest.TestCase):
    def test_world_selection_adds_matching_boxes_and_removes_stale_walls(self):
        config = load_config()
        all_names = {item['name'] for item in config['environment']}
        for world in ('open.world', 'cell.world'):
            for ground_enabled in (True, False):
                with self.subTest(world=world, ground_enabled=ground_enabled):
                    config['simulation']['world'] = 'assets/worlds/' + world
                    config['environment'][0]['enabled'] = ground_enabled
                    expected = {'ground_plane_user'} if world == 'open.world' else set(all_names)
                    if not ground_enabled:
                        expected.remove('ground_plane_user')
                    scene = Mock()
                    with patch.object(obstacles, 'load_config', return_value=config), \
                            patch.object(obstacles.rospy, 'init_node'), \
                            patch.object(obstacles.moveit_commander, 'roscpp_initialize'), \
                            patch.object(obstacles.moveit_commander, 'roscpp_shutdown'), \
                            patch.object(obstacles.moveit_commander, 'PlanningSceneInterface', return_value=scene):
                        obstacles.main()
                    self.assertEqual({call.args[0] for call in scene.add_box.call_args_list}, expected)
                    self.assertEqual({call.args[0] for call in scene.remove_world_object.call_args_list},
                                     all_names - expected)
                    for call in scene.add_box.call_args_list:
                        definition = next(item for item in config['environment'] if item['name'] == call.args[0])
                        self.assertEqual(call.kwargs['size'], definition['size'])
                        self.assertEqual(call.args[1].header.frame_id, 'world')
                        self.assertEqual(call.args[1].pose.position.z, definition['pose'][2])
                    self.assertEqual(scene.add_mesh.call_args.args[0], 'rect_object')
                    self.assertEqual(scene.add_mesh.call_args.args[1].pose.position.z, object_pose(config)[2])


if __name__ == '__main__':
    unittest.main()
