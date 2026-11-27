"""DefMapSim five-view camera geometry and configuration (metres)."""
import copy
import math

DEFAULTS = {
    'distance': 0.4, 'elevation_degrees': 30.0, 'orbit_yaw_degrees': 0.0,
    'roll_degrees': 180.0,
    'reference_offset_right': 0.035,
    'object_center_local': [-0.0004370113, -0.000512874, 0.0014823601],
    'mount_translation': [-0.02, -0.08, 0.0],
    'mount_forward': [0.9949708551, 0.0, 0.1001648512],
    'mount_up': [0.0, 0.0, 1.0],
    # Compact finger curl with the hand's distal coupling (J1 = 0.8 * J2).
    'fist_joint_angles': {
        'rh_FFJ1': 1.2, 'rh_FFJ2': 1.5, 'rh_FFJ3': 1.5, 'rh_FFJ4': 0.0,
        'rh_MFJ1': 1.2, 'rh_MFJ2': 1.5, 'rh_MFJ3': 1.5, 'rh_MFJ4': 0.0,
        'rh_RFJ1': 1.2, 'rh_RFJ2': 1.5, 'rh_RFJ3': 1.5, 'rh_RFJ4': 0.0,
        'rh_THJ1': 0.73, 'rh_THJ2': 0.6981317007977318,
        'rh_THJ4': 1.22, 'rh_THJ5': 0.05,
    },
    'ik_attempts': 64, 'ik_timeout': 0.05, 'planning_time': 5.0,
    'max_path_attempts': 40, 'pause_at_view': 1.0,
}
TASK_KIND = 'camera_views_v1'


def settings(config):
    from project_config import number, vector
    result = copy.deepcopy(DEFAULTS)
    supplied = config.get('camera_scan', {})
    if not isinstance(supplied, dict):
        raise ValueError('camera_scan must be a mapping')
    unknown = set(supplied) - set(result)
    if unknown:
        raise ValueError('Unknown camera_scan settings: %s' % sorted(unknown))
    result.update(supplied)
    for key in ('distance', 'ik_timeout', 'planning_time'):
        number(result[key], 'camera_scan.' + key, positive=True)
    for key in ('elevation_degrees', 'orbit_yaw_degrees', 'roll_degrees', 'reference_offset_right', 'pause_at_view'):
        number(result[key], 'camera_scan.' + key)
    if not 0 < result['elevation_degrees'] < 90 or result['pause_at_view'] < 0:
        raise ValueError('Camera elevation must be between 0 and 90 degrees; pause must be nonnegative')
    for key in ('ik_attempts', 'max_path_attempts'):
        if type(result[key]) is not int or not 1 <= result[key] <= 1000:
            raise ValueError('camera_scan.%s must be an integer from 1 to 1000' % key)
    for key in ('object_center_local', 'mount_translation', 'mount_forward', 'mount_up'):
        vector(result[key], 3, 'camera_scan.' + key)
    fist = result['fist_joint_angles']
    if not isinstance(fist, dict) or set(fist) != set(DEFAULTS['fist_joint_angles']):
        raise ValueError('camera_scan.fist_joint_angles must define all 16 hand joints')
    for name, value in fist.items():
        number(value, 'camera_scan.fist_joint_angles.' + name)
    f, u = result['mount_forward'], result['mount_up']
    cross = [f[1]*u[2]-f[2]*u[1], f[2]*u[0]-f[0]*u[2], f[0]*u[1]-f[1]*u[0]]
    if sum(v*v for v in cross) < 1e-12:
        raise ValueError('camera_scan mount_forward and mount_up must be nonzero and nonparallel')
    return result


def camera_targets(config):
    import numpy as np
    from tf.transformations import euler_matrix, quaternion_from_matrix
    from project_config import object_pose
    options = settings(config)
    obj = object_pose(config)
    center = np.asarray(obj[:3]) + euler_matrix(*obj[3:])[:3, :3].dot(options['object_center_local'])
    drop = euler_matrix(math.pi/2, 0, math.radians(options['orbit_yaw_degrees']))[:3, :3]
    forward = np.asarray(options['mount_forward'], dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, options['mount_up']); right /= np.linalg.norm(right)
    mount = np.eye(4)
    mount[:3, :3] = np.column_stack((right, np.cross(forward, right), forward))
    mount[:3, 3] = options['mount_translation']
    elevation = math.radians(options['elevation_degrees'])
    roll = euler_matrix(0, 0, math.radians(options['roll_degrees']))[:3, :3]
    targets = []
    for index in range(5):
        theta = 2*math.pi*index/5
        eye = options['distance']*np.array([math.cos(elevation)*math.cos(theta),
                                           math.sin(elevation), math.cos(elevation)*math.sin(theta)])
        forward = -eye/np.linalg.norm(eye)
        right = np.cross(forward, [0, 1, 0]); right /= np.linalg.norm(right)
        camera = np.eye(4)
        camera[:3, :3] = drop.dot(np.column_stack((right, np.cross(forward, right), forward)))
        camera[:3, 3] = center + drop.dot(eye-options['reference_offset_right']*right)
        # Roll around optical +Z after placing the calibrated optical center.
        # Preserve the view position and aim, then solve for the attached wrist.
        camera[:3, :3] = camera[:3, :3].dot(roll)
        flange = camera.dot(np.linalg.inv(mount))
        values = [*flange[:3, 3], *quaternion_from_matrix(flange)]
        targets.append({'index': index, 'azimuth_degrees': index*72,
                        'camera_optical_to_world': camera.tolist(),
                        'flange_pose': dict(zip(('x','y','z','qx','qy','qz','qw'), map(float, values)))})
    return {'center_world': center.tolist(), 'flange_from_optical': mount.tolist(), 'views': targets}
