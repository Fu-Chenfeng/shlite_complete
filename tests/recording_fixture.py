"""Create a valid five-stage file fixture; never executes any motion."""
import pickle

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from joint_targets import target_for_group
from task_definition import TASK_VERSION, STAGES, THUMB_PREPARATION_JOINTS, actions, save_task
from trajectory_store import create_run, finish_run


def make_recording(root, robot, groups, config):
    run = create_run(root, task_version=TASK_VERSION)
    save_task(run, config)
    hand_targets = {
        STAGES[0]['actions'][0]['file']: target_for_group(robot, groups['hand'],
            {name: value if name in THUMB_PREPARATION_JOINTS or name.endswith('J4') else value * config['planning'].get('approach_hand_fraction', 0.6)
             for name, value in config['grasp']['joint_angles'].items()}, True),
        STAGES[1]['actions'][0]['file']: target_for_group(robot, groups['hand'], config['grasp']['joint_angles'], True),
        STAGES[2]['actions'][0]['file']: target_for_group(robot, groups['hand'], config['grasp']['tightened_joint_angles'], True),
        STAGES[4]['actions'][1]['file']: target_for_group(robot, groups['hand'], {}, True),
    }
    for action in actions():
        names = groups[action['group']].get_active_joints()
        target = hand_targets.get(action['file'], {name: 0.0 for name in names})
        if action['file'] == 'Stage_5_Return_home.pkl':
            target = config['robot']['home_joints']
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(names)
        trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0.0] * len(names), time_from_start=rospy.Duration(0)),
            JointTrajectoryPoint(positions=[target[name] for name in names], time_from_start=rospy.Duration(1)),
        ]
        with open(run / action['group'] / action['file'], 'wb') as stream:
            pickle.dump(trajectory, stream)
        if action['group'] == 'hand':
            with open(run / 'hand' / action['target'], 'wb') as stream:
                pickle.dump(target, stream)
    finish_run(run, True)
    return run
