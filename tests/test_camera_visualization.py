"""Camera scene integration checks without a ROS master or robot motion."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml
from tf.transformations import quaternion_matrix

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from camera_views import camera_targets
from camera_visualization import GazeboCones, MARKER_TOPIC, configure_rviz
from project_config import load_config
import camera_visualization


class CameraVisualizationTest(unittest.TestCase):
    def test_scene_is_empty_until_camera_tour_publishes_targets(self):
        import rospy
        from std_msgs.msg import String
        from visualization_msgs.msg import Marker
        geometry = camera_targets(load_config())
        publisher, scene = Mock(), Mock()
        receivers = []

        def subscribe(topic, message_type, callback, **kwargs):
            receivers.append(callback)
            return Mock()

        def receive_after_startup(_):
            if len(scene.update.call_args_list) == 1:
                receivers[0](String(data=json.dumps(geometry)))

        with patch.object(sys, 'argv', ['camera_visualization.py']), \
                patch.dict('os.environ', {'SESSION_MODE': 'sim'}), \
                patch.object(rospy, 'init_node'), patch.object(rospy, 'get_param', return_value=True), \
                patch.object(rospy, 'Publisher', return_value=publisher), \
                patch.object(rospy, 'Subscriber', side_effect=subscribe), \
                patch.object(rospy, 'is_shutdown', side_effect=[False, False, True]), \
                patch.object(rospy.rostime, 'wallsleep', side_effect=receive_after_startup), \
                patch.object(camera_visualization, 'GazeboCones', return_value=scene):
            camera_visualization.main()

        self.assertEqual(scene.update.call_args_list[0].args[0], {'views': []})
        self.assertEqual(scene.update.call_args_list[1].args[0], geometry)
        initial, requested = [call.args[0].markers for call in publisher.publish.call_args_list]
        self.assertEqual([marker.action for marker in initial], [Marker.DELETEALL])
        self.assertEqual(sum(marker.action == Marker.ADD for marker in requested), 5)

    def test_rviz_keeps_existing_displays_and_adds_camera_display_once(self):
        original = {'Visualization Manager': {'Displays': [
            {'Class': 'moveit_rviz_plugin/MotionPlanning', 'Name': 'MotionPlanning'}],
            'Global Options': {'Fixed Frame': 'world'}}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'moveit.rviz'
            path.write_text(yaml.safe_dump(original))
            configure_rviz(path)
            configure_rviz(path)
            result = yaml.safe_load(path.read_text())['Visualization Manager']
        self.assertEqual(result['Displays'][0], original['Visualization Manager']['Displays'][0])
        self.assertEqual(result['Global Options'], {'Fixed Frame': 'world'})
        self.assertEqual(len(result['Displays']), 2)
        self.assertEqual(result['Displays'][1]['Marker Topic'], MARKER_TOPIC)
        self.assertEqual(result['Displays'][1]['Class'], 'rviz/MarkerArray')
        self.assertTrue(result['Displays'][1]['Enabled'])

    def test_gazebo_update_replaces_only_cones_and_sends_camera_world_poses(self):
        scene = GazeboCones.__new__(GazeboCones)
        scene.world, scene.delete, scene.spawn = Mock(), Mock(), Mock()
        scene.world.return_value.success = True
        scene.world.return_value.model_names = ['rect_object', 'robot', 'camera_view_cone_0']
        scene.delete.return_value.success = scene.spawn.return_value.success = True
        geometry = camera_targets(load_config())
        scene.update(geometry)
        scene.delete.assert_called_once_with('camera_view_cone_0')
        self.assertEqual(scene.spawn.call_count, 5)
        for view, call in zip(geometry['views'], scene.spawn.call_args_list):
            name, sdf, namespace, pose, frame = call.args
            self.assertEqual(name, 'camera_view_cone_%d' % view['index'])
            self.assertNotIn('<collision', sdf)
            self.assertEqual(frame, 'world')
            q = pose.orientation
            transform = quaternion_matrix([q.x, q.y, q.z, q.w])
            p = pose.position
            transform[:3, 3] = [p.x, p.y, p.z]
            np.testing.assert_allclose(transform, view['camera_optical_to_world'], atol=1e-12)
        scene.delete.reset_mock()
        scene.spawn.reset_mock()
        scene.update({'views': []})
        scene.delete.assert_called_once_with('camera_view_cone_0')
        scene.spawn.assert_not_called()


if __name__ == '__main__':
    unittest.main()
