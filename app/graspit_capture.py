"""Resolve Shadow Lite TXT/JSON captures into a world-frame palm pose."""
import copy
import hashlib
import json
import math

from grasp_pose import multiply, rotate, conjugate, unit

# GraspIt's exported hand base corresponds to this assembly's ra_flange.
# Fixed ra_flange -> rh_palm transform, verified against all captured link origins.
BASE_TO_PALM_TRANSLATION = [0.149, 0.0, 0.015]
BASE_TO_PALM_ORIENTATION = [-0.6408563820557884, 0.6408563820557887,
                            -0.2988362387301197, 0.2988362387301198]


def rpy_quaternion(roll, pitch, yaw):
    sr, cr = math.sin(roll/2), math.cos(roll/2)
    sp, cp = math.sin(pitch/2), math.cos(pitch/2)
    sy, cy = math.sin(yaw/2), math.cos(yaw/2)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def bounded(value, lower, upper, name, tolerance=1e-6):
    # Legacy capture writes six decimals; accept only that rounding error.
    if value < lower - tolerance or value > upper + tolerance:
        raise ValueError('GraspIt capture %s is outside the hand joint limits' % name)
    return min(upper, max(lower, value))


def numbers(values, count, label):
    if not isinstance(values, list) or len(values) != count or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('%s must contain %d finite numbers' % (label, count))
    return values


def json_pose(value, label):
    position = numbers(value['translation_mm'], 3, label + '.translation_mm')
    q = numbers(value['quaternion_xyzw'], 4, label + '.quaternion_xyzw')
    if abs(math.sqrt(sum(v*v for v in q))-1) > 0.01:
        raise ValueError(label + ' quaternion must have unit length')
    return [v*0.001 for v in position], unit(q)


def read_json_capture(data, index):
    document = json.loads(data)
    if not isinstance(document, dict) or document.get('schema_version') != 1 or document.get('coordinate_frame') != 'world':
        raise ValueError('Expected GraspIt capture JSON schema_version 1 with coordinate_frame world')
    captures = document['grasps']
    if not isinstance(captures, list) or index >= len(captures):
        raise ValueError('grasp.capture_index is outside the saved JSON grasps list')
    capture = captures[index]
    if not isinstance(capture, dict) or not isinstance(capture.get('hand_name'), str) or not capture['hand_name'].startswith('shadowhand_motor'):
        raise ValueError('JSON capture must use the shadowhand_motor model')
    dofs = numbers(capture['dof_values'], 13, 'GraspIt dof_values')
    hand_t, hand_q = json_pose(capture['hand_pose_world'], 'hand_pose_world')
    if 'object_pose_world' not in capture:
        raise ValueError('JSON capture needs object_pose_world to place the grasp relative to the configured object')
    object_t, object_q = json_pose(capture['object_pose_world'], 'object_pose_world')
    relative_t = rotate(conjugate(object_q), [a-b for a, b in zip(hand_t, object_t)])
    relative_q = multiply(conjugate(object_q), hand_q)
    values = [*[v*1000 for v in relative_t], relative_q[3], *relative_q[:3]]
    joints = None
    if 'joints' in capture:
        if not isinstance(capture['joints'], list) or len(capture['joints']) != 16:
            raise ValueError('JSON capture must include all 16 Shadow Lite joint values')
        joints = {}
        for entry in capture['joints']:
            chain, joint = entry['chain_index'], entry['joint_index']
            if type(chain) is not int or type(joint) is not int or not 0 <= chain < 4 or not 0 <= joint < 4:
                raise ValueError('Invalid Shadow Lite chain/joint index in JSON capture')
            if entry.get('type') != 'revolute':
                raise ValueError('Shadow Lite capture joints must be revolute')
            finger = ('FF', 'MF', 'RF', 'TH')[chain]
            number = ((4, 3, 2, 1) if chain < 3 else (5, 4, 2, 1))[joint]
            name = 'rh_%sJ%d' % (finger, number)
            if name in joints:
                raise ValueError('Duplicate JSON capture joint: ' + name)
            joints[name] = numbers([entry['joint_value_rad']], 1, name)[0]
        if len(joints) != 16:
            raise ValueError('JSON capture must include all 16 Shadow Lite joint values')
    gravity = capture.get('gravity_direction_world')
    gravity_palm = None
    if gravity is not None:
        gravity = unit(numbers(gravity, 3, 'gravity_direction_world'))
        gravity_palm = rotate(conjugate(multiply(hand_q, BASE_TO_PALM_ORIENTATION)), gravity)
    return dofs, values, joints, gravity, gravity_palm


def resolve_capture(grasp, object_pose, path):
    """Read the selected JSON grasp or the first legacy TXT grasp."""
    data = path.read_bytes()
    index = grasp.get('capture_index', 0)
    if type(index) is not int or index < 0:
        raise ValueError('grasp.capture_index must be a nonnegative integer')
    is_json = data.lstrip().startswith(b'{') or path.suffix.lower() == '.json'
    saved_joints = gravity = gravity_palm = None
    if is_json:
        dofs, values, saved_joints, gravity, gravity_palm = read_json_capture(data, index)
    else:
        if index:
            raise ValueError('Legacy TXT supports only capture_index 0; use JSON for multiple grasps')
        lines = [line.strip() for line in data.decode('utf-8').splitlines() if line.strip()]
        if len(lines) < 2:
            raise ValueError('GraspIt capture requires DOF and pose lines')
        posture, pose = lines[0].split(), lines[1].split()
        if len(posture) != 14 or posture[0] != '5' or len(pose) != 8 or pose[0] != '0':
            raise ValueError('Expected GraspIt Shadow Lite capture: 5 + 13 DOFs, then 0 + xyz + qw qx qy qz')
        dofs, values = list(map(float, posture[1:])), list(map(float, pose[1:]))
    if not all(math.isfinite(value) for value in [*dofs, *values]):
        raise ValueError('GraspIt capture contains nonfinite values')
    base_q = [*values[4:], values[3]]
    norm = math.sqrt(sum(value*value for value in base_q))
    if abs(norm - 1) > 0.01:
        raise ValueError('GraspIt capture quaternion must have unit length')
    base_q = [value/norm for value in base_q]
    palm_shift = rotate(base_q, BASE_TO_PALM_TRANSLATION)
    palm_position = [value*0.001 + shift for value, shift in zip(values[:3], palm_shift)]
    # Capture pose is object-relative. The object's configured pose performs
    # the same Y-up -> Z-up conversion as the imported mesh.
    object_q = rpy_quaternion(*object_pose[3:])
    position = [value+offset for value, offset in zip(rotate(object_q, palm_position), object_pose[:3])]
    orientation = multiply(object_q, multiply(base_q, BASE_TO_PALM_ORIENTATION))
    if orientation[3] < 0:
        orientation = [-value for value in orientation]
    joints, joint_limits = {}, {}
    for chain, finger in enumerate(('FF', 'MF', 'RF')):
        for joint, value, limits in zip((4, 3, 2, 1),
                (dofs[3*chain], dofs[3*chain+1], dofs[3*chain+2], 0.8*dofs[3*chain+2]),
                ((-20, 20), (-15, 90), (0, 90), (0, 90))):
            name = 'rh_%sJ%d' % (finger, joint)
            value = saved_joints[name] if saved_joints is not None else value
            joints[name] = bounded(value, *map(math.radians, limits), name, 1e-5 if is_json else 1e-6)
            joint_limits[name] = tuple(map(math.radians, limits))
    for joint, value, limits in zip((5, 4, 2, 1), dofs[9:],
                                   ((-60, 60), (0, 70), (-40, 40), (-15, 90))):
        name = 'rh_THJ%d' % joint
        value = saved_joints[name] if saved_joints is not None else value
        joints[name] = bounded(value, *map(math.radians, limits), name, 1e-5 if is_json else 1e-6)
        joint_limits[name] = tuple(map(math.radians, limits))
    bend = grasp.get('capture_tighten_degrees', 10.0)
    if isinstance(bend, bool) or not isinstance(bend, (int, float)) or not math.isfinite(bend):
        raise ValueError('grasp.capture_tighten_degrees must be a finite number')
    tightened = dict(joints)
    limited = {}
    for finger in ('FF', 'MF', 'RF', 'TH'):
        name = 'rh_%sJ2' % finger
        requested = joints[name] + math.radians(bend)
        lower, upper = joint_limits[name]
        tightened[name] = max(lower, min(upper, requested))
        if abs(tightened[name] - requested) > 1e-9:
            limited[name] = {'requested_degrees': bend,
                             'applied_degrees': math.degrees(tightened[name] - joints[name]),
                             'target_degrees': math.degrees(tightened[name])}
    resolved = copy.deepcopy(grasp)
    resolved.update(translation=position, orientation=orientation,
                    joint_angles=joints, tightened_joint_angles=tightened,
                    tightening_limited_joints=limited,
                    capture_sha256=hashlib.sha256(data).hexdigest())
    if gravity_palm is not None:
        resolved.update(gravity_direction_palm=unit(gravity_palm),
                        gravity_direction_graspit_world=gravity, capture_index=index)
    return resolved
