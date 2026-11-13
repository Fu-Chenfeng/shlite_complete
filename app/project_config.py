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
        number(sim.get('replay_speed', 3.0), 'simulation.replay_speed', positive=True)
        if 'physics_step' in sim:
            number(sim['physics_step'], 'simulation.physics_step', positive=True)
        selection = config.get('trajectories', {}).get('replay_run', 'latest')
        from trajectory_store import RUN_NAME
        if not isinstance(selection, str) or (selection != 'latest' and not RUN_NAME.fullmatch(selection)):
            raise ValueError('trajectories.replay_run must be latest or a saved folder name')
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
        number(obj.get('grasp_z_offset', 0.0), 'object.grasp_z_offset')
        for key in ('visual_size', 'collision_size', 'mesh_scale'):
            vector(obj[key], 3, 'object.' + key, positive=True)
        vector(obj['color'], 4, 'object.color')
        if any(value < 0 or value > 1 for value in obj['color']):
            raise ValueError('object.color values must be between 0 and 1')
        if obj['mesh'] is not None:
            project_path(obj['mesh'], root)
        boolean(obj.get('collision_mesh', False), 'object.collision_mesh')
        if obj.get('collision_mesh', False) and not obj['mesh']:
            raise ValueError('object.collision_mesh requires object.mesh')
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
        grasp = config['grasp']
        mode = grasp.get('mode', 'manual')
        if mode not in ('manual', 'graspit'):
            raise ValueError('grasp.mode must be manual or graspit')
        if mode == 'graspit':
            from graspit_capture import resolve_capture
            filename = grasp['capture_file']
            if not isinstance(filename, str) or not filename.strip():
                raise ValueError('grasp.capture_file must be a nonempty filename')
            config['grasp'] = resolve_capture(grasp, obj['pose'], project_path(filename, root))
        validate_planning(config['planning'], config['grasp'])
        object_model(config, root)
        return config
    except (KeyError, TypeError, AttributeError, OSError, yaml.YAMLError, ValueError, ET.ParseError) as exc:
        raise ValueError('Invalid config.yaml: %s' % exc) from exc


def validate_planning(plan, grasp):
    if grasp['frame'] != 'world':
        raise ValueError('grasp.frame must be world (translation and lift use world coordinates)')
    if grasp['hand_link'] != 'rh_palm':
        raise ValueError('grasp.hand_link must be rh_palm for this assembly')
    vector(grasp['translation'], 3, 'grasp.translation')
    vector(grasp['orientation'], 4, 'grasp.orientation')
    if abs(sum(value ** 2 for value in grasp['orientation']) - 1) > 0.01:
        raise ValueError('grasp.orientation must be a unit quaternion [x, y, z, w]')
    for key in ('joint_angles', 'tightened_joint_angles'):
        joints = grasp[key]
        if not isinstance(joints, dict) or not joints:
            raise ValueError('grasp.%s must be a nonempty joint map' % key)
        for name, value in joints.items():
            if not isinstance(name, str) or not name.startswith('rh_'):
                raise ValueError('grasp.%s must contain rh_ hand joint names' % key)
            number(value, 'grasp.%s.%s' % (key, name))
    if set(grasp['joint_angles']) != set(grasp['tightened_joint_angles']):
        raise ValueError('Both grasp joint maps must specify the same joints')
    boolean(plan.get('lift_align_gravity', False), 'planning.lift_align_gravity')
    if plan.get('lift_align_gravity', False):
        if 'gravity_direction_palm' not in grasp:
            raise ValueError('Gravity-aligned lift requires a JSON capture with gravity_direction_world '
                             'or an explicit grasp.gravity_direction_palm vector')
        vector(grasp['gravity_direction_palm'], 3, 'grasp.gravity_direction_palm')
        from grasp_pose import unit
        grasp['gravity_direction_palm'] = unit(grasp['gravity_direction_palm'], 'grasp.gravity_direction_palm')
    number(plan.get('lift_rotation_step_degrees', 2.0), 'planning.lift_rotation_step_degrees', positive=True)
    if plan.get('lift_rotation_step_degrees', 2.0) > 180:
        raise ValueError('planning.lift_rotation_step_degrees must be <= 180')
    for key in ('arm_velocity', 'arm_acceleration', 'hand_velocity', 'hand_acceleration'):
        number(plan[key], 'planning.' + key, positive=True)
        if plan[key] > 1:
            raise ValueError('planning.%s must be <= 1' % key)
    for key in ('lift_velocity', 'lift_acceleration'):
        if key in plan:
            number(plan[key], 'planning.' + key, positive=True)
            if plan[key] > 1:
                raise ValueError('planning.%s must be <= 1' % key)
    fraction = plan.get('approach_hand_fraction', 0.6)
    number(fraction, 'planning.approach_hand_fraction')
    if not 0 <= fraction <= 1:
        raise ValueError('planning.approach_hand_fraction must be between 0 and 1')
    for key in ('planning_time', 'joint_tolerance', 'lift_distance', 'lift_step',
                'lift_min_fraction', 'lift_min_duration'):
        number(plan[key], 'planning.' + key, positive=True)
    if plan['lift_min_fraction'] > 1:
        raise ValueError('planning.lift_min_fraction must be <= 1')
    if type(plan['planning_attempts']) is not int or plan['planning_attempts'] < 1:
        raise ValueError('planning.planning_attempts must be a positive integer')
    attempts = plan.get('approach_ik_attempts', 64)
    if type(attempts) is not int or not 1 <= attempts <= 1000:
        raise ValueError('planning.approach_ik_attempts must be an integer between 1 and 1000')
    number(plan.get('approach_ik_timeout', 0.1), 'planning.approach_ik_timeout', positive=True)


def text_vector(values):
    return ' '.join(str(value) for value in values)


def object_pose(config):
    values = list(config['object']['pose'])
    values[2] += config['object'].get('grasp_z_offset', 0.0)
    return values


def grasp_target(config):
    """Offset only the task target; retain the original capture/YAML coordinates."""
    grasp = copy.deepcopy(config['grasp'])
    grasp['translation'][2] += config['object'].get('grasp_z_offset', 0.0)
    return grasp


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
    collision = model.find('link/collision/geometry')
    collision.clear()
    if obj.get('collision_mesh', False):
        mesh = ET.SubElement(collision, 'mesh')
        path = project_path(obj['mesh'], root).relative_to(root)
        ET.SubElement(mesh, 'uri').text = 'file:///project/' + path.as_posix()
        ET.SubElement(mesh, 'scale').text = text_vector(obj['mesh_scale'])
    else:
        ET.SubElement(ET.SubElement(collision, 'box'), 'size').text = text_vector(obj['collision_size'])
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


def world_model_names(config, world=None, root=ROOT):
    """Model names selected by the shared simulation/real environment world."""
    scene = ET.parse(project_path(world or config['simulation']['world'], root)).getroot().find('world')
    if scene is None:
        raise ValueError('Simulation world must be an SDF containing a world')
    return {element.findtext('name') if element.tag == 'include' else element.get('name')
            for element in scene if element.tag in ('include', 'model')}


def prepare_simulation(config, world=None, root=ROOT):
    """Inline configured models so vendor launch model paths cannot override them."""
    tree = ET.parse(project_path(world or config['simulation']['world'], root))
    scene = tree.getroot().find('world')
    if scene is None:
        raise ValueError('Simulation world must be an SDF containing a world')
    if 'physics_step' in config['simulation']:
        physics = scene.find('physics')
        if physics is None:
            physics = ET.SubElement(scene, 'physics', type='ode', name='configured_physics')
        step = config['simulation']['physics_step']
        for name, value in (('max_step_size', step), ('real_time_update_rate', 1.0 / step)):
            field = physics.find(name)
            if field is None:
                field = ET.SubElement(physics, name)
            field.text = str(value)
    if config['object']['enabled'] and not config['object'].get('hand_collision', True):
        plugin = ET.SubElement(scene, 'plugin', name='shlite_hand_object_collisions',
                               filename='/tmp/shlite-contact-filter/libhand_object_collisions.so')
        ET.SubElement(plugin, 'object_name').text = 'rect_object'
    settings = {item['name']: item for item in config['environment']}
    settings['rect_object'] = config['object']
    # Remove managed instances; reinsert only environment models present in this world.
    present = world_model_names(config, world, root)
    for element in list(scene):
        name = element.findtext('name') if element.tag == 'include' else element.get('name')
        if name in settings:
            scene.remove(element)
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
        pose.text = text_vector(object_pose(config) if name == 'rect_object' else definition['pose'])
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
