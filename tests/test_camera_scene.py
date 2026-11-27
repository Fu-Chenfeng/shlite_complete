"""Camera visual placement and mesh geometry; no running ROS master required."""
from collections import Counter
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from tf.transformations import euler_matrix, quaternion_matrix
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
from camera_scene import (ALPHA, LENGTH, MESH_URI, RADIUS, clear_markers,
                          cone_model_name, cone_sdf, cone_triangles, view_markers)
from camera_views import camera_targets
from project_config import load_config


class CameraSceneTest(unittest.TestCase):
    def setUp(self):
        self.geometry = camera_targets(load_config())

    def test_cone_tip_is_at_each_camera_and_base_follows_its_optical_axis(self):
        markers = view_markers(self.geometry).markers
        self.assertEqual(len(markers), 5)
        self.assertEqual([marker.id for marker in markers], list(range(5)))
        for view, marker in zip(self.geometry['views'], markers):
            camera = np.array(view['camera_optical_to_world'])
            orientation = marker.pose.orientation
            transform = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
            point = marker.pose.position
            transform[:3, 3] = [point.x, point.y, point.z]
            np.testing.assert_allclose(transform, camera, atol=1e-12)
            tip = transform.dot([0, 0, 0, 1])[:3]
            base = transform.dot([0, 0, marker.scale.z, 1])[:3]
            np.testing.assert_allclose(tip, camera[:3, 3], atol=1e-12)
            np.testing.assert_allclose(base-tip, LENGTH*camera[:3, 2], atol=1e-12)
            self.assertEqual((marker.scale.x, marker.scale.y), (RADIUS, RADIUS))
            self.assertEqual(marker.header.frame_id, 'world')
            self.assertEqual(marker.type, Marker.MESH_RESOURCE)
            self.assertEqual(marker.mesh_resource, MESH_URI)
            self.assertFalse(marker.mesh_use_embedded_materials)
            self.assertEqual((marker.color.r, marker.color.g, marker.color.b), (1, 1, 0))
            self.assertAlmostEqual(marker.color.a, ALPHA)
            self.assertGreater(marker.color.a, 0)
            self.assertLess(marker.color.a, 1)

    def test_gazebo_cones_have_no_collision_geometry_or_extra_pose_offset(self):
        for view in self.geometry['views']:
            root = ET.fromstring(cone_sdf(view))
            model = root.find('model')
            self.assertEqual(model.get('name'), cone_model_name(view))
            self.assertEqual(model.findtext('static'), 'true')
            self.assertEqual(model.findtext('link/gravity'), 'false')
            self.assertEqual(root.findall('.//collision'), [])
            pose = [float(value) for value in model.findtext('pose').split()]
            transform = euler_matrix(*pose[3:])
            transform[:3, 3] = pose[:3]
            np.testing.assert_allclose(transform, np.eye(4), atol=1e-12)
            visual = model.find('link/visual')
            self.assertAlmostEqual(float(visual.findtext('transparency')), 1-ALPHA)
            self.assertEqual(visual.findtext('geometry/mesh/uri'), MESH_URI)
            np.testing.assert_allclose([float(value) for value in visual.findtext('geometry/mesh/scale').split()],
                                       [RADIUS, RADIUS, LENGTH])
            self.assertEqual(visual.findtext('material/diffuse'), '1 1 0 1')

    def test_unit_mesh_is_closed_with_outward_normals_and_a_round_base(self):
        triangles = cone_triangles()
        self.assertEqual(len(triangles), 96)
        directed_edges = Counter()
        for triangle in triangles:
            a, b, c = np.asarray(triangle)
            normal = np.cross(b-a, c-a)
            self.assertGreater(np.linalg.norm(normal), 0)
            if np.allclose(a, [0, 0, 1]):
                self.assertGreater(normal[2], 0)
            else:
                self.assertEqual(tuple(a), (0, 0, 0))
                self.assertLess(normal[2], 0)
                self.assertGreater(np.dot(normal[:2], (a+b+c)[:2]), 0)
            for index in range(3):
                directed_edges[(triangle[index], triangle[(index+1) % 3])] += 1
        for (a, b), count in directed_edges.items():
            self.assertEqual(count, 1)
            self.assertEqual(directed_edges[(b, a)], 1)
        vertices = {point for triangle in triangles for point in triangle}
        rim = [point for point in vertices if point[2] == 1 and point[:2] != (0, 0)]
        self.assertEqual(len(rim), 48)
        np.testing.assert_allclose([np.linalg.norm(point[:2]) for point in rim], 1, atol=1e-12)

    def test_disabled_scene_can_clear_stale_cones(self):
        markers = clear_markers().markers
        self.assertEqual(len(markers), 1)
        self.assertEqual(markers[0].action, Marker.DELETEALL)


if __name__ == '__main__':
    unittest.main()
