import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
from grasp_pose import flange_pose, multiply, rotate
from project_config import load_config


class GraspPoseTest(unittest.TestCase):
    def test_hand_translation_is_converted_using_rotated_flange_offset(self):
        q = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        grasp = {'translation': [1, 2, 3], 'orientation': q}
        flange = flange_pose(grasp, [0.2, 0, 0.1], [0, 0, 0, 1])
        self.assertAlmostEqual(flange['x'], 1)
        self.assertAlmostEqual(flange['y'], 1.8)
        self.assertAlmostEqual(flange['z'], 2.9)

    def test_pose_round_trip_with_rotated_palm_mount(self):
        mount_q = [-0.6408563820557884, 0.6408563820557887, -0.2988362387301197, 0.2988362387301198]
        mount_t = [0.149, 0, 0.015]
        grasp = load_config()['grasp']
        flange = flange_pose(grasp, mount_t, mount_q)
        q = [flange[k] for k in ('qx', 'qy', 'qz', 'qw')]
        actual = [flange[k] + d for k, d in zip(('x', 'y', 'z'), rotate(q, mount_t))]
        for expected, value in zip(grasp['translation'], actual):
            self.assertAlmostEqual(expected, value)
        for expected, value in zip(grasp['orientation'], multiply(q, mount_q)):
            self.assertAlmostEqual(expected, value)

    def test_capture_tightening_adds_configured_degrees_only_to_penultimate_joints(self):
        grasp = load_config()['grasp']
        joints = {'rh_FFJ2', 'rh_MFJ2', 'rh_RFJ2', 'rh_THJ2'}
        degrees = grasp.get('capture_tighten_degrees', 10)
        for name, value in grasp['joint_angles'].items():
            delta = grasp['tightened_joint_angles'][name] - value
            target = value
            if name in joints:
                lower, upper = (-40, 40) if name == 'rh_THJ2' else (0, 90)
                target = max(math.radians(lower), min(math.radians(upper), value + math.radians(degrees)))
            self.assertAlmostEqual(delta, target - value)
        self.assertGreater(load_config()['planning']['lift_distance'], 0)


if __name__ == '__main__':
    unittest.main()
