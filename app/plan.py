#!/usr/bin/env python3
import sys, os, re, copy, pickle, math
import rospy, moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, CollisionObject
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel
from geometry_msgs.msg import Pose as GazeboPose

TRAJECTORY_ROOT = os.environ.get("TRAJECTORY_ROOT", "/project/trajectories")
ARM_TRAJ_DIR = os.path.join(TRAJECTORY_ROOT, "arm")
HAND_TRAJ_DIR = os.path.join(TRAJECTORY_ROOT, "hand")

ARM_POSE_STAGE2 = {'x': 0.560912011, 'y': 0.041459514, 'z': 0.276758128, 'qx': -0.06858597323858148, 'qy': 0.9148256430460816, 'qz': 0.02719998938689261, 'qw': 0.3970518450751649}
ARM_POSE_STAGE4 = {'x': 0.560912011, 'y': 0.041459514, 'z': 0.276758128, 'qx': -0.06858597323858148, 'qy': 0.9148256430460816, 'qz': 0.02719998938689261, 'qw': 0.3970518450751649}
ARM_POSE_STAGE5_PRELIFT = {'x': 0.26320091199999995, 'y': 0.041459514, 'z': 0.410871051, 'qx': -0.06773090371496682, 'qy': 0.36612136366582065, 'qz': -0.029264309827985665, 'qw': 0.9276374679365456}
HAND_JOINTS_CAPTURED = {'rh_FFJ1': 0.093444493509459, 'rh_FFJ2': 0.093444493509459, 'rh_FFJ3': 1.432007902592064, 'rh_FFJ4': 0.348716784548, 'rh_MFJ1': 0.01372251375, 'rh_MFJ2': 0.01372251375, 'rh_MFJ3': 1.567656304937532, 'rh_MFJ4': 0.0, 'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.567656304937532, 'rh_RFJ4': -0.348716784548, 'rh_THJ1': 0.338699416551993, 'rh_THJ2': 0.389480010701418, 'rh_THJ4': 1.22050874592, 'rh_THJ5': -0.238956313567}
HAND_JOINTS_ITERATED = {'rh_FFJ1': 0.7873341836350589, 'rh_FFJ2': 0.093444493509459, 'rh_FFJ3': 1.432007902592064, 'rh_FFJ4': 0.348716784548, 'rh_MFJ1': 0.828737374703355, 'rh_MFJ2': 0.01372251375, 'rh_MFJ3': 1.567656304937532, 'rh_MFJ4': 0.0, 'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 1.567656304937532, 'rh_RFJ4': -0.348716784548, 'rh_THJ1': 1.163699755315065, 'rh_THJ2': 0.389480010701418, 'rh_THJ4': 1.22050874592, 'rh_THJ5': -0.238956313567}
HAND_JOINT_DELTAS_ITERATED = {'rh_FFJ1': {'from': 0.093444493509459, 'to': 0.7873341836350589, 'delta': 0.6938896901255999, 'change': '+0.693889690126'}, 'rh_MFJ1': {'from': 0.01372251375, 'to': 0.828737374703355, 'delta': 0.815014860953355, 'change': '+0.815014860953'}, 'rh_THJ1': {'from': 0.338699416551993, 'to': 1.163699755315065, 'delta': 0.825000338763072, 'change': '+0.825000338763'}}
STAGE4_CHANGED_ANGLE_SCALE = 0.5

STAGE4_IGNORE_ENV = True
STAGE4_TOL = 0.001
STAGE4_PLANNING_TIME = 20.0
STAGE4_ATTEMPTS = 5

STAGE5_Z_LIFT = 0.05
STAGE5_EEF_STEP = 0.005
STAGE5_MIN_FRACTION = 0.99
STAGE5_DURATION = 5.0

HOME_JOINTS = {'ra_shoulder_pan_joint': -0.873, 'ra_shoulder_lift_joint': -2.094, 'ra_elbow_joint': 2.094, 'ra_wrist_1_joint': -0.873, 'ra_wrist_2_joint': -0.611, 'ra_wrist_3_joint': -3.142}
THUMB_ONLY = ("rh_THJ2", "rh_THJ4", "rh_THJ5")


def pose(d):
    p = Pose()
    p.position.x, p.position.y, p.position.z = d["x"], d["y"], d["z"]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = d["qx"], d["qy"], d["qz"], d["qw"]
    return p


def plan(group):
    p = group.plan()
    return p[1] if isinstance(p, tuple) else p


def save_traj(traj, label, group_name):
    base = ARM_TRAJ_DIR if group_name == "arm" else HAND_TRAJ_DIR
    os.makedirs(base, exist_ok=True)
    name = re.sub(r'/\d+', '', label).replace("/", "_").replace(" ", "_").replace(":", "")
    path = os.path.join(base, name + ".pkl")
    with open(path, "wb") as f:
        pickle.dump(traj, f)
    rospy.loginfo("[%s] saved %s", group_name, path)


def save_joint_delta(delta, label):
    """Save only the hand joints changed by the iterated solution, before planning."""
    os.makedirs(HAND_TRAJ_DIR, exist_ok=True)
    name = re.sub(r'/\d+', '', label).replace("/", "_").replace(" ", "_").replace(":", "")
    path = os.path.join(HAND_TRAJ_DIR, name + ".pkl")
    with open(path, "wb") as f:
        pickle.dump(delta, f)
    rospy.loginfo("[hand] saved iterated joint delta %s", path)


def save_hand_target_joints(joints, label):
    """Save the target hand joint map before planning."""
    os.makedirs(HAND_TRAJ_DIR, exist_ok=True)
    name = re.sub(r'/\d+', '', label).replace("/", "_").replace(" ", "_").replace(":", "")
    path = os.path.join(HAND_TRAJ_DIR, name + "_target_joints.pkl")
    with open(path, "wb") as f:
        pickle.dump(dict(joints), f)
    rospy.loginfo("[hand] saved target joints %s", path)


def joint_limits(robot, name):
    j = robot.get_joint(name)
    for getter in (lambda: j.bounds(), lambda: (j.min_bound(), j.max_bound())):
        try:
            b = getter()
            if b is not None:
                return float(b[0]), float(b[1])
        except Exception:
            pass
    return None, None


def clamp(robot, joints, label):
    out = dict(joints)
    for name, val in joints.items():
        mn, mx = joint_limits(robot, name)
        if mn is not None and mx is not None:
            out[name] = max(mn, min(mx, float(val)))
            if abs(out[name] - float(val)) > 1e-12:
                rospy.logwarn("%s clamped %s: %.12f -> %.12f", label, name, float(val), out[name])
    return out


def allow_robot_world_collisions(robot):
    rospy.wait_for_service("/get_planning_scene")
    rospy.wait_for_service("/apply_planning_scene")
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)

    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.WORLD_OBJECT_NAMES
    scene = get_scene(req).scene
    original = copy.deepcopy(scene.allowed_collision_matrix)
    objects = [o.id for o in scene.world.collision_objects if o.id]
    links = robot.get_link_names()

    if not objects:
        return lambda: None

    acm = copy.deepcopy(original)

    def ensure(name):
        if name in acm.entry_names:
            return
        acm.entry_names.append(name)
        for row in acm.entry_values:
            row.enabled.append(False)
        row = AllowedCollisionEntry()
        row.enabled = [False] * len(acm.entry_names)
        acm.entry_values.append(row)

    def set_allowed(a, b):
        ensure(a); ensure(b)
        ia, ib = acm.entry_names.index(a), acm.entry_names.index(b)
        acm.entry_values[ia].enabled[ib] = True
        acm.entry_values[ib].enabled[ia] = True

    for obj in objects:
        for link in links:
            set_allowed(obj, link)

    diff = PlanningScene(is_diff=True)
    diff.allowed_collision_matrix = acm
    apply_scene(ApplyPlanningSceneRequest(scene=diff))
    rospy.logwarn("Allowed robot-vs-world collisions for %d objects", len(objects))

    def restore():
        diff = PlanningScene(is_diff=True)
        diff.allowed_collision_matrix = original
        apply_scene(ApplyPlanningSceneRequest(scene=diff))

    return restore




def _planning_scene_world_object_names():
    rospy.wait_for_service("/get_planning_scene")
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    return get_scene(req).scene


def _rect_object_candidate_names(scene=None):
    """Return only names related to rect_object; do not touch any other object."""
    if scene is None:
        try:
            scene = _planning_scene_world_object_names()
        except Exception:
            scene = None
    names = set(["rect_object", "rect_object::link", "rect_object::link::collision"])
    if scene is not None:
        for o in scene.world.collision_objects:
            if "rect_object" in o.id:
                names.add(o.id)
        for n in scene.allowed_collision_matrix.entry_names:
            if "rect_object" in n:
                names.add(n)
    return sorted(names)


def remove_rect_object_from_moveit(robot, timeout=5.0):
    """Disable MoveIt collision checking for rect_object before Stage 5a.

    This removes only rect_object-related collision objects from the planning scene
    and also allows robot-vs-rect_object names in the ACM while the scene update
    propagates. Other objects and self-collision settings are left unchanged.
    """
    rospy.wait_for_service("/get_planning_scene")
    rospy.wait_for_service("/apply_planning_scene")
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)

    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    scene = get_scene(req).scene
    original_acm = copy.deepcopy(scene.allowed_collision_matrix)
    rect_names = _rect_object_candidate_names(scene)
    robot_links = robot.get_link_names()

    acm = copy.deepcopy(original_acm)

    def ensure(name):
        if name in acm.entry_names:
            return
        acm.entry_names.append(name)
        for row in acm.entry_values:
            row.enabled.append(False)
        row = AllowedCollisionEntry()
        row.enabled = [False] * len(acm.entry_names)
        acm.entry_values.append(row)

    def set_allowed(a, b):
        ensure(a); ensure(b)
        ia, ib = acm.entry_names.index(a), acm.entry_names.index(b)
        acm.entry_values[ia].enabled[ib] = True
        acm.entry_values[ib].enabled[ia] = True

    for rect in rect_names:
        for link in robot_links:
            set_allowed(rect, link)

    diff = PlanningScene(is_diff=True)
    diff.allowed_collision_matrix = acm
    for rect in rect_names:
        co = CollisionObject()
        co.id = rect
        co.operation = CollisionObject.REMOVE
        diff.world.collision_objects.append(co)
    apply_scene(ApplyPlanningSceneRequest(scene=diff))
    rospy.logwarn("Disabled MoveIt collision checking for rect_object before Stage 5a; names=%s", rect_names)

    # Wait until all known rect_object world collision objects are gone from MoveIt's planning scene.
    deadline = rospy.Time.now() + rospy.Duration.from_sec(timeout)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        now_scene = get_scene(req).scene
        remaining = [o.id for o in now_scene.world.collision_objects if "rect_object" in o.id]
        if not remaining:
            rospy.logwarn("Confirmed rect_object removed from MoveIt world objects before Stage 5a")
            break
        rospy.sleep(0.05)
    else:
        rospy.logwarn("rect_object may still be in MoveIt world objects after wait; continuing with ACM allowed")

    def restore():
        restore_diff = PlanningScene(is_diff=True)
        restore_diff.allowed_collision_matrix = original_acm
        apply_scene(ApplyPlanningSceneRequest(scene=restore_diff))
        rospy.loginfo("Restored MoveIt allowed collision matrix after Stage 5a")

    return restore


def _find_rect_object_model_xml():
    paths = []
    for base in os.environ.get("GAZEBO_MODEL_PATH", "").split(":"):
        if base:
            paths.append(os.path.join(base, "rect_object", "model.sdf"))
    paths.extend([
        "/project/assets/models/rect_object/model.sdf",
        "/home/user/.gazebo/models/rect_object/model.sdf",
        "/home/user/rect_object/model.sdf",
        "/usr/share/gazebo/models/rect_object/model.sdf",
    ])
    for path in paths:
        if os.path.exists(path):
            with open(path) as f:
                return f.read()
    return None


def disable_rect_object_in_gazebo(timeout=5.0):
    """Disable only rect_object physical collision in Gazebo for Stage 5a.

    Gazebo's standard ROS API does not expose a pairwise runtime collision-bitmask
    service, so this deletes only the rect_object model before the stage and respawns
    it afterward at the same pose. No other Gazebo model is changed.
    """
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/delete_model")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    state = get_state("rect_object", "world")
    if not getattr(state, "success", False):
        rospy.logwarn("rect_object not found in Gazebo; nothing to disable")
        return lambda: None

    model_xml = _find_rect_object_model_xml()
    pose_saved = copy.deepcopy(state.pose)
    delete_model("rect_object")
    rospy.logwarn("Disabled Gazebo physical collision for rect_object before Stage 5a by deleting only that model")

    def restore():
        if not model_xml:
            rospy.logwarn("Cannot respawn rect_object: model.sdf was not found. Restore it manually if needed.")
            return
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        try:
            spawn_model("rect_object", model_xml, "", pose_saved, "world")
            rospy.loginfo("Restored Gazebo rect_object model after Stage 5a")
        except Exception as e:
            rospy.logerr("Failed to respawn rect_object: %s", e)

    return restore

def force_trajectory_start_to_current(group, traj, label):
    """Avoid MoveIt Invalid Trajectory start-state mismatch.
    Some hand joints keep settling after planning; MoveIt compares trajectory
    point 0 against the live state at execute time.
    """
    names = list(traj.joint_trajectory.joint_names)
    current_names = group.get_active_joints()
    current_vals = group.get_current_joint_values()
    current = dict(zip(current_names, current_vals))

    if not traj.joint_trajectory.points:
        return traj

    p0 = traj.joint_trajectory.points[0]
    p0.positions = [float(current[n]) for n in names]
    p0.velocities = [0.0] * len(names)
    p0.accelerations = [0.0] * len(names)
    p0.time_from_start = rospy.Duration.from_sec(0.0)
    rospy.loginfo("%s: synced trajectory start point to current robot state", label)
    return traj


def execute_pose(arm, pose_dict, label):
    rospy.loginfo(label)
    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose(pose_dict))
    traj = plan(arm)
    if not traj or not traj.joint_trajectory.points:
        raise RuntimeError(label + " planning failed")
    save_traj(traj, label, "arm")
    ok = arm.execute(traj, wait=True)
    arm.stop(); arm.clear_pose_targets()
    if not ok:
        raise RuntimeError(label + " execute failed")


def execute_hand(hand, joints, label, save_target=True):
    rospy.loginfo(label)
    hand.stop()
    rospy.sleep(0.2)

    hand.set_start_state_to_current_state()
    if save_target:
        save_hand_target_joints(joints, label)
    hand.set_joint_value_target(joints)
    traj = plan(hand)
    if not traj or not traj.joint_trajectory.points:
        raise RuntimeError(label + " planning failed")

    # Important for Shadow Hand: joints can still settle between planning and execution.
    # MoveIt rejects the trajectory if point 0 differs from the live joint state by > allowed_start_tolerance.
    traj = force_trajectory_start_to_current(hand, traj, label)

    save_traj(traj, label, "hand")
    if not hand.execute(traj, wait=True):
        raise RuntimeError(label + " execute failed")
    hand.stop()
    rospy.sleep(0.2)


def stretch_time(traj, duration):
    pts = traj.joint_trajectory.points
    if not pts:
        return traj
    old_final = pts[-1].time_from_start.to_sec()
    if old_final <= 1e-9:
        for i, p in enumerate(pts):
            p.time_from_start = rospy.Duration.from_sec(duration * i / max(1, len(pts) - 1))
    else:
        scale = float(duration) / old_final
        for p in pts:
            p.time_from_start = rospy.Duration.from_sec(p.time_from_start.to_sec() * scale)
            p.velocities = [v / scale for v in p.velocities]
            p.accelerations = [a / (scale * scale) for a in p.accelerations]
    return traj


def execute_cartesian_z_lift(arm, label):
    rospy.loginfo(label)
    start = arm.get_current_pose().pose
    target = copy.deepcopy(start)
    target.position.z += STAGE5_Z_LIFT
    traj, fraction = arm.compute_cartesian_path([target], STAGE5_EEF_STEP, 0.0)
    if not traj or not traj.joint_trajectory.points or fraction < STAGE5_MIN_FRACTION:
        raise RuntimeError("%s Cartesian planning failed: fraction=%.3f" % (label, fraction))
    traj = stretch_time(traj, STAGE5_DURATION)
    traj = force_trajectory_start_to_current(arm, traj, label)
    save_traj(traj, label, "arm")
    if not arm.execute(traj, wait=True):
        raise RuntimeError(label + " execute failed")
    arm.stop(); arm.clear_pose_targets()


def nearest_equivalent_angle(current, target):
    return current + math.atan2(math.sin(target - current), math.cos(target - current))


def execute_joint_plan(arm, robot, target, label):
    rospy.loginfo(label)
    target = clamp(robot, target, label)

    current = dict(zip(arm.get_active_joints(), arm.get_current_joint_values()))
    target = {
        name: nearest_equivalent_angle(current[name], float(value)) if name in current else float(value)
        for name, value in target.items()
    }
    target = clamp(robot, target, label)

    arm.set_start_state_to_current_state()
    arm.set_joint_value_target(target)

    traj = plan(arm)
    if not traj or not traj.joint_trajectory.points:
        raise RuntimeError(label + " planning failed")

    save_traj(traj, label, "arm")

    if not arm.execute(traj, wait=True):
        raise RuntimeError(label + " execute failed")

    arm.stop()
    arm.clear_pose_targets()


def thumb_only(full):
    return {k: float(full[k]) if k in THUMB_ONLY else 0.0 for k in full}


def scale_iterated_joints(captured, iterated, scale):
    return {
        k: float(captured.get(k, 0.0)) + (float(iterated.get(k, 0.0)) - float(captured.get(k, 0.0))) * float(scale)
        for k in sorted(set(captured) | set(iterated))
    }


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur5_shadowhand_from_testgrasp", anonymous=True)
    if os.environ.get("SESSION_MODE") != "sim" or not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("Planning-and-saving workflow must run in the simulation session")
    robot = moveit_commander.RobotCommander()
    hand = moveit_commander.MoveGroupCommander("right_hand")
    arm = moveit_commander.MoveGroupCommander("right_arm")

    hand.set_max_velocity_scaling_factor(0.4)
    hand.set_max_acceleration_scaling_factor(0.4)
    arm.set_max_velocity_scaling_factor(0.08)
    arm.set_max_acceleration_scaling_factor(0.08)
    arm.set_planning_time(60.0)

    execute_hand(hand, {k: 0.0 for k in HAND_JOINTS_CAPTURED}, "Stage 0: Open hand")
    execute_hand(hand, clamp(robot, thumb_only(HAND_JOINTS_CAPTURED), "Stage 1"), "Stage 1/5: Hand thumb-only")
    execute_pose(arm, ARM_POSE_STAGE2, "Stage 2/5: Move arm to captured hand pose")

    restore = allow_robot_world_collisions(robot) if STAGE4_IGNORE_ENV else (lambda: None)
    try:
        target = {k: float(v) for k, v in HAND_JOINTS_CAPTURED.items()}
        execute_hand(hand, clamp(robot, target, "Stage 3 all fingers"), "Stage 3/5: Move all fingers")
    finally:
        restore()

    restore = allow_robot_world_collisions(robot) if STAGE4_IGNORE_ENV else (lambda: None)
    try:
        hand.set_goal_joint_tolerance(STAGE4_TOL)
        hand.set_planning_time(STAGE4_PLANNING_TIME)
        hand.set_num_planning_attempts(STAGE4_ATTEMPTS)
        hand.set_max_velocity_scaling_factor(0.1)
        hand.set_max_acceleration_scaling_factor(0.1)
        execute_pose(arm, ARM_POSE_STAGE4, "Stage 4/5: Move arm with palm shift")
        save_joint_delta(HAND_JOINT_DELTAS_ITERATED, "Stage 4/5: Iterated hand joint angle changes")
        stage4_scaled_joints = scale_iterated_joints(HAND_JOINTS_CAPTURED, HAND_JOINTS_ITERATED, STAGE4_CHANGED_ANGLE_SCALE)
        execute_hand(hand, clamp(robot, stage4_scaled_joints, "Stage 4 hand"), "Stage 4/5: Apply iterated hand joint angles", save_target=False)
    finally:
        restore()

    restore_moveit_rect = remove_rect_object_from_moveit(robot)
    restore_gazebo_rect = disable_rect_object_in_gazebo()
    try:
        execute_pose(arm, ARM_POSE_STAGE5_PRELIFT, "Stage 5a/5: Move arm to additional offset pose")
    finally:
        restore_gazebo_rect()
        restore_moveit_rect()

    restore = allow_robot_world_collisions(robot) if STAGE4_IGNORE_ENV else (lambda: None)
    try:
        execute_cartesian_z_lift(arm, "Stage 5/5: Straight-line Z lift")
        execute_joint_plan(arm, robot, HOME_JOINTS, "After Stage 5: Planned joint move to home")
    finally:
        restore()

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
