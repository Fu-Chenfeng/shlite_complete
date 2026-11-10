"""Shared configuration and generated Gazebo scene. No ROS dependencies."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def inside(path, parent):
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def project_path(value, root=ROOT):
    """Translate container paths too; every asset must travel with the project."""
    path = Path(value)
    if inside(path, Path('/project')):
        path = root / path.relative_to('/project')
    elif not path.is_absolute():
        path = root / path
    path = path.resolve()
    if not inside(path, root.resolve()) or not path.is_file():
        raise ValueError("Project file does not exist inside the project: %s" % value)
    return path


def number(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s must be a finite number" % label)
    if positive and value <= 0:
        raise ValueError("%s must be positive" % label)


def vector(value, length, label, positive=False):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError("%s must contain %d numbers" % (label, length))
    for item in value:
        number(item, label, positive)


def boolean(value, label):
    if not isinstance(value, bool):
        raise ValueError("%s must be true or false" % label)


def load_config(root=ROOT):
    try:
        config = yaml.safe_load((root / 'config.yaml').read_text())
        network, sim, robot, obj = (config[key] for key in ('network', 'simulation', 'robot', 'object'))
        for key in ('pc_ip', 'nuc_ip', 'nuc_user', 'nuc_container', 'ssh_key', 'ur5_ip'):
            if not isinstance(network[key], str) or not network[key].strip():
                raise ValueError('network.%s must be a nonempty string' % key)
        if not isinstance(network['ur5_reverse_ip'], str):
            raise ValueError('network.ur5_reverse_ip must be a string')
        port = network['ros_port']
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('network.ros_port must be between 1 and 65535')
        project_path(sim['world'], root)
        boolean(sim['gui'], 'simulation.gui')
        selection = config.get('trajectories', {}).get('replay_run', 'latest')
        from trajectory_store import RUN_NAME
        if not isinstance(selection, str) or (selection != 'latest' and not RUN_NAME.fullmatch(selection)):
            raise ValueError('trajectories.replay_run must be latest or a timestamp folder name')
        for key in ('initial_y', 'initial_z'):
            number(robot[key], 'robot.' + key)
        expected_joints = {'ra_%s_joint' % joint for joint in (
            'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')}
        if set(robot['home_joints']) != expected_joints:
            raise ValueError('robot.home_joints must define all six UR5 arm joints')
        for value in robot['home_joints'].values():
            number(value, 'robot.home_joints')
        project_path(obj['model'], root)
        for key in ('enabled', 'static', 'add_to_real_scene'):
            boolean(obj[key], 'object.' + key)
        boolean(obj.get('hand_collision', True), 'object.hand_collision')
        number(obj['mass'], 'object.mass', positive=True)
        vector(obj['pose'], 6, 'object.pose')
        for key in ('visual_size', 'collision_size', 'mesh_scale'):
            vector(obj[key], 3, 'object.' + key, positive=True)
        vector(obj['color'], 4, 'object.color')
        if any(value < 0 or value > 1 for value in obj['color']):
            raise ValueError('object.color values must be between 0 and 1')
        if obj['mesh'] is not None:
            project_path(obj['mesh'], root)
        names = set()
        for obstacle in config['environment']:
            name = obstacle['name']
            if name in names or name == 'rect_object':
                raise ValueError('Duplicate/reserved environment name: ' + name)
            names.add(name)
            project_path('assets/models/%s/model.sdf' % name, root)
            boolean(obstacle['enabled'], name + '.enabled')
            vector(obstacle['size'], 3, name + '.size', positive=True)
            vector(obstacle['pose'], 6, name + '.pose')
        validate_planning(config['planning'])
        object_model(config, root)
        return config
    except (KeyError, TypeError, AttributeError, OSError, yaml.YAMLError, ValueError, ET.ParseError) as exc:
        raise ValueError('Invalid config.yaml: %s' % exc) from exc


def validate_planning(plan):
    for key in ('arm_pose_stage2', 'arm_pose_stage4', 'arm_pose_stage5_prelift'):
        pose = plan[key]
        if set(pose) != {'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}:
            raise ValueError('planning.%s must contain x/y/z/qx/qy/qz/qw' % key)
        for value in pose.values():
            number(value, 'planning.' + key)
        if abs(sum(pose[key] ** 2 for key in ('qx', 'qy', 'qz', 'qw')) - 1) > 0.01:
            raise ValueError('Planning pose quaternion must have unit length')
    for key in ('hand_joints_captured', 'hand_joints_iterated'):
        if not plan[key]:
            raise ValueError('planning.%s cannot be empty' % key)
        for value in plan[key].values():
            number(value, 'planning.' + key)
    for delta in plan['hand_joint_deltas_iterated'].values():
        for key in ('from', 'to', 'delta'):
            number(delta[key], 'planning.hand_joint_deltas_iterated.' + key)
    boolean(plan['stage4_ignore_env'], 'planning.stage4_ignore_env')
    for key in ('stage4_changed_angle_scale', 'stage4_tol', 'stage4_planning_time',
                'stage5_z_lift', 'stage5_eef_step', 'stage5_min_fraction', 'stage5_duration'):
        number(plan[key], 'planning.' + key, positive=True)
    if plan['stage5_min_fraction'] > 1:
        raise ValueError('planning.stage5_min_fraction must be <= 1')
    if type(plan['stage4_attempts']) is not int or plan['stage4_attempts'] < 1:
        raise ValueError('planning.stage4_attempts must be a positive integer')
    if not plan['thumb_only'] or not all(isinstance(j, str) for j in plan['thumb_only']):
        raise ValueError('planning.thumb_only must list joint names')


def text_vector(values):
    return ' '.join(str(value) for value in values)


def set_text(parent, path, text):
    element = parent.find(path)
    if element is None:
        raise ValueError('Model is missing <%s>' % path)
    element.text = str(text)


def object_model(config, root=ROOT):
    obj = config['object']
    model = ET.parse(project_path(obj['model'], root)).getroot().find('model')
    if model is None:
        raise ValueError('object.model must be an SDF containing a model')
    model.set('name', 'rect_object')
    set_text(model, 'static', str(obj['static']).lower())
    set_text(model, 'link/inertial/mass', obj['mass'])
    set_text(model, 'link/collision/geometry/box/size', text_vector(obj['collision_size']))
    visual = model.find('link/visual')
    geometry = visual.find('geometry')
    geometry.clear()
    if obj['mesh']:
        mesh = ET.SubElement(geometry, 'mesh')
        path = project_path(obj['mesh'], root).relative_to(root)
        ET.SubElement(mesh, 'uri').text = 'file:///project/' + path.as_posix()
        ET.SubElement(mesh, 'scale').text = text_vector(obj['mesh_scale'])
    else:
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = text_vector(obj['visual_size'])
    for field in ('ambient', 'diffuse'):
        set_text(visual, 'material/' + field, text_vector(obj['color']))
    return model


def prepare_simulation(config, world=None, root=ROOT):
    """Inline configured models so vendor launch model paths cannot override them."""
    tree = ET.parse(project_path(world or config['simulation']['world'], root))
    scene = tree.getroot().find('world')
    if scene is None:
        raise ValueError('Simulation world must be an SDF containing a world')
    if config['object']['enabled'] and not config['object'].get('hand_collision', True):
        plugin = ET.SubElement(scene, 'plugin', name='shlite_hand_object_collisions',
                               filename='/tmp/shlite-contact-filter/libhand_object_collisions.so')
        ET.SubElement(plugin, 'object_name').text = 'rect_object'
    settings = {item['name']: item for item in config['environment']}
    settings['rect_object'] = config['object']
    # Remove managed instances; reinsert only environment models present in this world.
    present = set()
    for element in list(scene):
        name = element.findtext('name') if element.tag == 'include' else element.get('name')
        if name in settings:
            scene.remove(element)
            present.add(name)
    present.add('rect_object')
    obj_model = object_model(config, root)
    runtime = root / '.runtime'
    (runtime / 'models/rect_object').mkdir(parents=True, exist_ok=True)
    sdf = ET.Element('sdf', version='1.4')
    sdf.append(copy.deepcopy(obj_model))
    ET.ElementTree(sdf).write(runtime / 'models/rect_object/model.sdf', encoding='utf-8', xml_declaration=True)
    for name, definition in settings.items():
        if name not in present or not definition['enabled']:
            continue
        if name == 'rect_object':
            model = copy.deepcopy(obj_model)
        else:
            model = ET.parse(root / 'assets/models' / name / 'model.sdf').getroot().find('model')
            for part in ('collision', 'visual'):
                set_text(model, 'link/%s/geometry/box/size' % part, text_vector(definition['size']))
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.text = text_vector(definition['pose'])
        scene.append(model)
    output = runtime / 'simulation.world'
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def launch_arguments(config, mode):
    robot = config['robot']
    arguments = ['initial_z:=' + str(robot['initial_z']), 'initial_y:=' + str(robot['initial_y'])]
    if mode == 'sim':
        arguments += ['gui:=' + str(config['simulation']['gui']).lower(),
                      'home_angles:=' + ' '.join('-J %s %s' % item for item in robot['home_joints'].items())]
    return arguments
