"""ROS-free custom sequence model and recording contract (angles in radians)."""
import copy
from datetime import datetime
import json
import math
from pathlib import Path

from trajectory_store import TOKYO, RUN_NAME

TASK_VERSION = 'custom_v1'
RVIZ_GOAL_TOPIC = '/custom_trajectory/rviz_goal'


def default_name(now=None):
    return (now or datetime.now(TOKYO)).astimezone(TOKYO).strftime('custom_%Y-%m-%d_%H-%M-%S')


def validate_sequence(sequence):
    if not isinstance(sequence, dict) or sequence.get('version') != TASK_VERSION:
        raise ValueError('Unsupported custom trajectory format')
    name = sequence.get('name')
    if not isinstance(name, str) or not RUN_NAME.fullmatch(name) or name == 'latest':
        raise ValueError('Enter a trajectory name using letters, numbers, spaces, _, -, . or parentheses')
    tasks = sequence.get('tasks')
    if not isinstance(tasks, list) or not tasks:
        raise ValueError('Add at least one task to the sequence')
    expected = None
    for index, task in enumerate(tasks, 1):
        if not isinstance(task, dict) or not isinstance(task.get('name'), str) or not task['name'].strip():
            raise ValueError('Task %d needs a name' % index)
        if task.get('order') not in ('arm_hand', 'hand_arm'):
            raise ValueError('Task %d needs an arm/hand motion order' % index)
        names = {}
        for group in ('arm', 'hand'):
            joints = task.get(group)
            if not isinstance(joints, dict) or not joints:
                raise ValueError('Task %d needs complete %s joint angles' % (index, group))
            for joint, angle in joints.items():
                if (not isinstance(joint, str) or not joint or isinstance(angle, bool)
                        or not isinstance(angle, (int, float)) or not math.isfinite(angle)):
                    raise ValueError('Invalid joint angle in task %d' % index)
            names[group] = set(joints)
        if names['arm'] & names['hand']:
            raise ValueError('Arm and hand joints must be distinct')
        if expected is not None and names != expected:
            raise ValueError('Every task must specify the same arm and hand joints')
        expected = names
    return sequence


def stages(sequence):
    validate_sequence(sequence)
    result = []
    for index, task in enumerate(sequence['tasks'], 1):
        actions = []
        for group in task['order'].split('_'):
            stem = 'Custom_%03d_%s' % (index, group)
            action = {'group': group, 'file': stem + '.pkl'}
            if group == 'hand':
                action['target'] = stem + '_target_joints.pkl'
            actions.append(action)
        result.append({'name': task['name'], 'actions': actions})
    return result


def load_task(run):
    task = json.loads((Path(run) / 'task.json').read_text())
    validate_sequence(task)
    if task.get('stages') != stages(task):
        raise ValueError('Custom recording has an invalid action sequence')
    options = task.get('planning', {})
    tolerance = options.get('joint_tolerance')
    if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
            or not math.isfinite(tolerance) or tolerance <= 0):
        raise ValueError('Custom recording has an invalid joint tolerance')
    return task


class Sequence:
    def __init__(self, state):
        self.initial = copy.deepcopy({group: state[group] for group in ('arm', 'hand')})
        self.tasks = []
        self.recordings = []

    def add(self):
        previous = self.tasks[-1] if self.tasks else self.initial
        task = {group: copy.deepcopy(previous[group]) for group in ('arm', 'hand')}
        task.update(name='Task %d' % (len(self.tasks) + 1), order='arm_hand')
        self.tasks.append(task)
        return task

    def capture(self, state):
        recorded = copy.deepcopy(state)
        recorded['name'] = 'State %d' % (len(self.recordings) + 1)
        recorded['captured_at'] = datetime.now(TOKYO).isoformat()
        self.recordings.append(recorded)
        return recorded

    def document(self, name):
        return copy.deepcopy(validate_sequence({'version': TASK_VERSION, 'name': name.strip(), 'tasks': self.tasks}))


def configure_rviz(path):
    import yaml
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    for display in config.get('Visualization Manager', {}).get('Displays', []):
        if display.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
            display['Class'] = 'shlite_custom_rviz/MotionPlanning'
    path.write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    import sys
    if len(sys.argv) == 3 and sys.argv[1] == 'configure-rviz':
        configure_rviz(sys.argv[2])
    else:
        raise SystemExit('Usage: custom_trajectory.py configure-rviz PATH')
