"""Validate complete joint targets before sending any robot motion."""
import math


def thumb_base_target(robot, hand, angles, tolerance=0.01):
    """Prepare the requested thumb-base joints, holding the other live angles."""
    if not isinstance(angles, dict) or not angles or set(angles) - {'rh_THJ5', 'rh_THJ4'}:
        raise ValueError('Thumb preparation must target rh_THJ5 and/or rh_THJ4')
    names = hand.get_active_joints()
    values = hand.get_current_joint_values()
    if len(values) != len(names) or not set(angles).issubset(names):
        raise ValueError('Complete hand state including the thumb-base joints is required for preparation')
    target = dict(zip(names, values))
    for name, value in target.items():
        lower, upper = robot.get_joint(name).bounds()
        if not math.isfinite(value) or value < lower-tolerance or value > upper+tolerance:
            raise ValueError('Current hand state is outside joint limits: ' + name)
        # Gazebo feedback can drift slightly past a mechanical stop.
        target[name] = max(lower, min(upper, value))
    target.update(angles)
    return target_for_group(robot, hand, target)


def approach_hand_target(robot, hand, grasp, fraction=0.6, tolerance=0.01):
    """Fully prepare J4 and thumb base; blend other joints from live to grasp."""
    if isinstance(fraction, bool) or not isinstance(fraction, (int, float)) or not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('Approach hand fraction must be between 0 and 1')
    grasp = target_for_group(robot, hand, grasp)
    bases = ('rh_THJ5', 'rh_THJ4')
    target = thumb_base_target(robot, hand, {name: grasp[name] for name in bases}, tolerance)
    for name in target:
        if name in bases or name in ('rh_FFJ4', 'rh_MFJ4', 'rh_RFJ4'):
            target[name] = grasp[name]
        else:
            target[name] += fraction * (grasp[name] - target[name])
    return target_for_group(robot, hand, target)


def target_for_group(robot, group, requested, fill_zero=False):
    active = group.get_active_joints()
    unknown = set(requested) - set(active)
    if unknown:
        raise ValueError('Target contains joints outside the group: %s' % sorted(unknown))
    target = {name: 0.0 for name in active} if fill_zero else {}
    target.update(requested)
    if set(target) != set(active):
        raise ValueError('Target must specify every joint in the group')
    for name, value in target.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Invalid joint angle for ' + name)
        bounds = robot.get_joint(name).bounds()
        if value < bounds[0] - 1e-9 or value > bounds[1] + 1e-9:
            raise ValueError('%s target %.6f is outside limits [%.6f, %.6f]' % (name, value, *bounds))
        target[name] = max(bounds[0], min(bounds[1], float(value)))
    return target
