"""Shared five-stage task contract for saving and replaying recordings."""
import copy
import json
from pathlib import Path

TASK_VERSION = 3
THUMB_PREPARATION_JOINTS = ['rh_THJ5', 'rh_THJ4']
STAGES = [
    {'name': 'Prepare thumb base, then approach hand grasp pose', 'actions': [
        {'group': 'hand', 'file': 'Stage_1_Prepare_thumb_base.pkl', 'target': 'Stage_1_Prepare_thumb_base_target_joints.pkl'},
        {'group': 'arm', 'file': 'Stage_1_Approach_grasp.pkl'}]},
    {'name': 'Close fingers to grasp angles', 'actions': [
        {'group': 'hand', 'file': 'Stage_2_Close_fingers.pkl', 'target': 'Stage_2_Close_fingers_target_joints.pkl'}]},
    {'name': 'Apply tightened grasp angles', 'actions': [
        {'group': 'hand', 'file': 'Stage_3_Tighten_grasp.pkl', 'target': 'Stage_3_Tighten_grasp_target_joints.pkl'}]},
    {'name': 'Lift hand', 'actions': [
        {'group': 'arm', 'file': 'Stage_4_Lift.pkl'}]},
    {'name': 'Return arm home, then zero hand', 'actions': [
        {'group': 'arm', 'file': 'Stage_5_Return_home.pkl'},
        {'group': 'hand', 'file': 'Stage_5_Open_hand.pkl', 'target': 'Stage_5_Open_hand_target_joints.pkl'}]},
]


def save_task(run, config):
    from project_config import object_pose
    task = {'version': TASK_VERSION, 'stages': copy.deepcopy(STAGES),
            'grasp': copy.deepcopy(config['grasp']),
            'home_joints': copy.deepcopy(config['robot']['home_joints']),
            'planning': copy.deepcopy(config['planning']),
            'thumb_preparation_joints': list(THUMB_PREPARATION_JOINTS),
            'hand_preparation_mode': 'saved_target',
            'scene': {'object_pose': object_pose(config),
                      'object_grasp_z_offset': config['object'].get('grasp_z_offset', 0.0)}}
    if config['planning'].get('lift_align_gravity', False):
        from grasp_pose import aligned_orientation
        task['lift_alignment'] = {
            'gravity_direction_palm': copy.deepcopy(config['grasp']['gravity_direction_palm']),
            'target_direction_world': [0, 0, -1],
            'target_palm_orientation': aligned_orientation(config['grasp']['orientation'],
                                                          config['grasp']['gravity_direction_palm'])}
    (Path(run) / 'task.json').write_text(json.dumps(task, indent=2) + '\n')
    return task


def load_task(run):
    try:
        task = json.loads((Path(run) / 'task.json').read_text())
    except (OSError, ValueError) as exc:
        raise ValueError('This recording uses the old task format. Generate a new five-stage run with Plan, execute & save.') from exc
    if not isinstance(task, dict) or task.get('version') != TASK_VERSION or task.get('stages') != STAGES:
        raise ValueError('Recording does not match the current five-stage task format')
    thumb_preparation_joints(task)
    if task.get('hand_preparation_mode') not in (None, 'saved_target'):
        raise ValueError('Invalid recorded hand preparation mode')
    return task


def actions():
    for stage in STAGES:
        yield from stage['actions']


def thumb_preparation_joints(task):
    # Older version-3 recordings prepared only THJ5.
    names = task.get('thumb_preparation_joints', ['rh_THJ5'])
    if names not in (['rh_THJ5'], THUMB_PREPARATION_JOINTS):
        raise ValueError('Invalid recorded thumb preparation joints')
    return names
