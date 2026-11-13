"""Verify the recorded palm displacement and final state after simulation."""
import pickle

import rospy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header

from task_definition import load_task
from arm_replay import angle_error


def verify_recording(run, groups, config):
    task = load_task(run)
    with open(run / 'arm/Stage_4_Lift.pkl', 'rb') as stream:
        trajectory = pickle.load(stream).joint_trajectory
    fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

    def palm(point):
        state = RobotState()
        state.joint_state.name = list(trajectory.joint_names)
        state.joint_state.position = list(point.positions)
        for name, value in task['grasp']['tightened_joint_angles'].items():
            state.joint_state.name.append(name)
            state.joint_state.position.append(value)
        response = fk(header=Header(frame_id='world'), fk_link_names=['rh_palm'], robot_state=state)
        assert response.error_code.val == 1, response
        return response.pose_stamped[0].pose

    start, end = (palm(point) for point in (trajectory.points[0], trajectory.points[-1]))
    displacement = end.position.z - start.position.z
    assert abs(displacement - task['planning']['lift_distance']) < 0.002, displacement
    assert abs(end.position.x - start.position.x) < 0.002
    assert abs(end.position.y - start.position.y) < 0.002
    if task['planning'].get('lift_align_gravity', False):
        from grasp_pose import rotate
        q = end.orientation
        gravity = rotate([q.x, q.y, q.z, q.w], task['grasp']['gravity_direction_palm'])
        assert gravity[2] < -0.999, gravity
        print('Verified final captured gravity direction is aligned with world -Z.', flush=True)
    for name, actual in zip(groups['arm'].get_active_joints(), groups['arm'].get_current_joint_values()):
        assert angle_error(actual, config['robot']['home_joints'][name]) < 0.03, (name, actual)
    for name, actual in zip(groups['hand'].get_active_joints(), groups['hand'].get_current_joint_values()):
        assert abs(actual) < 0.03, (name, actual)
    print('Verified recorded palm lift %.4f m and final arm home / hand zero.' % displacement, flush=True)
