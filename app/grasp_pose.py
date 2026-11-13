"""Rigid pose conversion from the configured hand frame to the arm flange."""
import math


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return [w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s]


def conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(q, vector):
    return multiply(multiply(q, [*vector, 0]), conjugate(q))[:3]


def unit(values, label='vector'):
    if not isinstance(values, (list, tuple)) or not values or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('%s must contain finite numbers' % label)
    norm = math.sqrt(sum(v*v for v in values))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('%s must be nonzero' % label)
    return [v/norm for v in values]


def rotation_between(first, second):
    a, b = unit(first), unit(second)
    dot = max(-1.0, min(1.0, sum(x*y for x, y in zip(a, b))))
    if dot < -1 + 1e-10:
        basis = [0.0]*3
        basis[min(range(3), key=lambda i: abs(a[i]))] = 1.0
        cross = [a[1]*basis[2]-a[2]*basis[1], a[2]*basis[0]-a[0]*basis[2], a[0]*basis[1]-a[1]*basis[0]]
        return [*unit(cross), 0.0]
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    return unit([*cross, 1.0+dot], 'rotation quaternion')


def aligned_orientation(orientation, gravity_palm):
    """Shortest world rotation that carries the captured gravity vector to -Z."""
    q = unit(orientation, 'hand orientation')
    delta = rotation_between(rotate(q, unit(gravity_palm)), [0, 0, -1])
    return unit(multiply(delta, q), 'aligned hand orientation')


def slerp(first, second, fraction):
    a, b = unit(first), unit(second)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = [-v for v in b], -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return unit([x+fraction*(y-x) for x, y in zip(a, b)])
    angle = math.acos(dot)
    return [(math.sin((1-fraction)*angle)*x + math.sin(fraction*angle)*y)/math.sin(angle)
            for x, y in zip(a, b)]


def palm_pose(flange, mount_translation, mount_orientation):
    q = [flange[k] for k in ('qx', 'qy', 'qz', 'qw')]
    shift = rotate(q, mount_translation)
    return {'translation': [flange[k]+v for k, v in zip(('x', 'y', 'z'), shift)],
            'orientation': multiply(q, mount_orientation)}


def mount_from_poses(flange, palm):
    q = [flange[k] for k in ('qx', 'qy', 'qz', 'qw')]
    delta = [palm['translation'][i]-flange[k] for i, k in enumerate(('x', 'y', 'z'))]
    return rotate(conjugate(q), delta), multiply(conjugate(q), palm['orientation'])


def lift_endpoint(flange, grasp, options):
    if not options.get('lift_align_gravity', False):
        result = dict(flange)
        result['z'] += options['lift_distance']
        return result
    mount_t, mount_q = mount_from_poses(flange, grasp)
    target = {'translation': list(grasp['translation']),
              'orientation': aligned_orientation(grasp['orientation'], grasp['gravity_direction_palm'])}
    target['translation'][2] += options['lift_distance']
    return flange_pose(target, mount_t, mount_q)


def lift_waypoints(start, distance, translation_step, gravity_palm, angular_step):
    final = aligned_orientation(start['orientation'], gravity_palm)
    dot = abs(sum(x*y for x, y in zip(unit(start['orientation']), final)))
    angle = 2*math.acos(min(1.0, dot))
    count = max(1, math.ceil(distance/translation_step), math.ceil(angle/angular_step))
    if count > 10000:
        raise ValueError('Lift sampling requests too many waypoints; increase the lift step sizes')
    return [{'translation': [*start['translation'][:2], start['translation'][2]+distance*i/count],
             'orientation': slerp(start['orientation'], final, i/count)} for i in range(1, count+1)]


def flange_pose(grasp, offset_translation, offset_orientation):
    """T_world_flange = T_world_hand * inverse(T_flange_hand)."""
    orientation = multiply(grasp['orientation'], conjugate(offset_orientation))
    shift = rotate(orientation, offset_translation)
    translation = [value - delta for value, delta in zip(grasp['translation'], shift)]
    return dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*translation, *orientation]))
