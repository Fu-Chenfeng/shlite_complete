"""Apply the configured hand-object collision rule to MoveIt."""
import copy
from contextlib import contextmanager

import rospy
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, ApplyPlanningScene, ApplyPlanningSceneRequest

from project_config import load_config


def contact_matrix(original, links, object_name='rect_object', allowed=True):
    matrix = copy.deepcopy(original)
    defaults = dict(zip(original.default_entry_names, original.default_entry_values))

    def default_allowed(first, second):
        values = [defaults[name] for name in (first, second) if name in defaults]
        return all(values) if values else False

    for name in [object_name, *links]:
        if name not in matrix.entry_names:
            existing = list(matrix.entry_names)
            matrix.entry_names.append(name)
            for other, entry in zip(existing, matrix.entry_values):
                entry.enabled.append(default_allowed(name, other))
            matrix.entry_values.append(AllowedCollisionEntry(
                enabled=[default_allowed(name, other) for other in matrix.entry_names]))
    index = matrix.entry_names.index(object_name)
    for name in links:
        other = matrix.entry_names.index(name)
        matrix.entry_values[index].enabled[other] = allowed
        matrix.entry_values[other].enabled[index] = allowed
    return matrix


def hand_object_matrix(original, links, config, object_names=()):
    hand_links = [name for name in links if name.startswith('rh_')]
    if not hand_links:
        raise RuntimeError('Combined robot description has no rh_ hand links')
    names = {'rect_object', 'rect_object::link', 'rect_object::link::collision',
             'rect_object__link', 'rect_object__link__collision'}
    names.update(name for name in [*original.entry_names, *object_names]
                 if name.startswith(('rect_object::', 'rect_object__')))
    matrix = original
    allowed = not config['object'].get('hand_collision', True)
    for name in sorted(names):
        matrix = contact_matrix(matrix, hand_links, name, allowed=allowed)
    return matrix


def configure_hand_object_contacts(robot, config=None):
    config = config if config is not None else load_config()
    if not config['object']['enabled']:
        return lambda: None
    rospy.wait_for_service('/get_planning_scene', timeout=10)
    rospy.wait_for_service('/apply_planning_scene', timeout=10)
    get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    request = GetPlanningSceneRequest()
    request.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                     PlanningSceneComponents.WORLD_OBJECT_NAMES)
    scene = get_scene(request).scene
    original = copy.deepcopy(scene.allowed_collision_matrix)
    updated = hand_object_matrix(original, robot.get_link_names(), config,
                                 [obj.id for obj in scene.world.collision_objects])

    def apply(matrix):
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        diff.allowed_collision_matrix = matrix
        if not apply_scene(ApplyPlanningSceneRequest(scene=diff)).success:
            raise RuntimeError('Could not update the hand-object collision rule')

    apply(updated)
    rospy.loginfo('Hand-object collision checking: %s',
                  'enabled' if config['object'].get('hand_collision', True) else 'disabled')

    def restore():
        if not rospy.is_shutdown():
            apply(original)

    return restore


@contextmanager
def stage_one_contacts(robot, config=None):
    """Temporarily require hand-object checking, restoring even on failure."""
    config = copy.deepcopy(config if config is not None else load_config())
    config['object']['hand_collision'] = True
    restore = configure_hand_object_contacts(robot, config)
    try:
        yield
    finally:
        restore()


def disable_replay_collision_checks(robot, protect_table=True):
    """Allow replay contacts, retaining robot-table checking for real control."""
    for service in ('/get_planning_scene', '/apply_planning_scene'):
        rospy.wait_for_service(service, timeout=10)
    get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    request = GetPlanningSceneRequest()
    request.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                     PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    scene = get_scene(request).scene
    original = copy.deepcopy(scene.allowed_collision_matrix)
    tables = [obj.id for obj in scene.world.collision_objects
              if obj.id == 'ground_plane_user' or obj.id.startswith(('ground_plane_user__', 'ground_plane_user::'))]
    if protect_table and not tables:
        raise RuntimeError('Real replay requires the table (ground_plane_user) in the planning scene. '
                           'Load the real-control scene before replay.')
    names = sorted(set(robot.get_link_names()) | set(original.entry_names) |
                   set(original.default_entry_names) |
                   {obj.id for obj in scene.world.collision_objects} |
                   {obj.object.id for obj in scene.robot_state.attached_collision_objects})
    allowed = AllowedCollisionMatrix(
        entry_names=names,
        entry_values=[AllowedCollisionEntry(enabled=[True] * len(names)) for _ in names],
        default_entry_names=names, default_entry_values=[True] * len(names))
    if protect_table:
        for table in tables:
            allowed = contact_matrix(allowed, names, table, allowed=False)
            allowed.default_entry_values[allowed.default_entry_names.index(table)] = False
            # Never restore a previously disabled robot-table rule on exit.
            original = contact_matrix(original, robot.get_link_names() +
                                      [obj.object.id for obj in scene.robot_state.attached_collision_objects],
                                      table, allowed=False)
            if table not in original.default_entry_names:
                original.default_entry_names.append(table)
                original.default_entry_values.append(False)
            else:
                original.default_entry_values[original.default_entry_names.index(table)] = False

    def apply(matrix):
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        diff.allowed_collision_matrix = matrix
        if not apply_scene(ApplyPlanningSceneRequest(scene=diff)).success:
            raise RuntimeError('Could not update replay collision rules')

    apply(allowed)
    rospy.loginfo('Replay collision checking: %s',
                  'robot-table enabled; other pairs disabled' if protect_table else 'disabled (simulation)')
    return lambda: apply(original)


def main():
    import moveit_commander
    rospy.init_node('shlite_hand_object_collisions', anonymous=True)
    moveit_commander.roscpp_initialize([])
    try:
        configure_hand_object_contacts(moveit_commander.RobotCommander())
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
