"""Temporary grasp-object contact allowance for Gazebo replay only."""
import copy
import os

import rospy
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, ApplyPlanningScene, ApplyPlanningSceneRequest


def contact_matrix(original, links, object_name="rect_object"):
    matrix = copy.deepcopy(original)
    for name in [object_name, *links]:
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
    index = matrix.entry_names.index(object_name)
    for name in links:
        other = matrix.entry_names.index(name)
        matrix.entry_values[index].enabled[other] = True
        matrix.entry_values[other].enabled[index] = True
    return matrix


def allow_grasp_contacts(robot):
    if os.environ.get("SESSION_MODE") != "sim" or not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("Simulated grasp-contact allowance requires the simulation session")
    rospy.wait_for_service("/get_planning_scene", timeout=10)
    rospy.wait_for_service("/apply_planning_scene", timeout=10)
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
    request = GetPlanningSceneRequest()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    original = get_scene(request).scene.allowed_collision_matrix
    updated = contact_matrix(original, robot.get_link_names())

    def apply(matrix):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        if not apply_scene(ApplyPlanningSceneRequest(scene=scene)).success:
            raise RuntimeError("Could not update the simulated grasp-contact matrix")

    apply(updated)

    def restore():
        if not rospy.is_shutdown():
            apply(original)

    return restore
