#!/usr/bin/env python3
import sys, os, re, copy, pickle, math
import rospy, moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry

from project_config import load_config
from sim_contacts import configure_hand_object_contacts, hand_object_matrix
from trajectory_store import archive_root, create_run, finish_run

PROJECT_CONFIG = load_config()

TRAJECTORY_ROOT = str(archive_root())
ARM_TRAJ_DIR = None
HAND_TRAJ_DIR = None


def start_recording():
    global ARM_TRAJ_DIR, HAND_TRAJ_DIR
    run = create_run(TRAJECTORY_ROOT)
    ARM_TRAJ_DIR, HAND_TRAJ_DIR = str(run / 'arm'), str(run / 'hand')
    rospy.loginfo('Saving this planning run to %s (Asia/Tokyo)', run)
    return run


def ensure_recording():
    if ARM_TRAJ_DIR is None or HAND_TRAJ_DIR is None:
        start_recording()

ARM_POSE_STAGE2 = PROJECT_CONFIG["planning"]["arm_pose_stage2"]
ARM_POSE_STAGE4 = PROJECT_CONFIG["planning"]["arm_pose_stage4"]
ARM_POSE_STAGE5_PRELIFT = PROJECT_CONFIG["planning"]["arm_pose_stage5_prelift"]
HAND_JOINTS_CAPTURED = PROJECT_CONFIG["planning"]["hand_joints_captured"]
HAND_JOINTS_ITERATED = PROJECT_CONFIG["planning"]["hand_joints_iterated"]
HAND_JOINT_DELTAS_ITERATED = PROJECT_CONFIG["planning"]["hand_joint_deltas_iterated"]
STAGE4_CHANGED_ANGLE_SCALE = PROJECT_CONFIG["planning"]["stage4_changed_angle_scale"]

STAGE4_IGNORE_ENV = PROJECT_CONFIG["planning"]["stage4_ignore_env"]
STAGE4_TOL = PROJECT_CONFIG["planning"]["stage4_tol"]
STAGE4_PLANNING_TIME = PROJECT_CONFIG["planning"]["stage4_planning_time"]
STAGE4_ATTEMPTS = PROJECT_CONFIG["planning"]["stage4_attempts"]

STAGE5_Z_LIFT = PROJECT_CONFIG["planning"]["stage5_z_lift"]
STAGE5_EEF_STEP = PROJECT_CONFIG["planning"]["stage5_eef_step"]
STAGE5_MIN_FRACTION = PROJECT_CONFIG["planning"]["stage5_min_fraction"]
STAGE5_DURATION = PROJECT_CONFIG["planning"]["stage5_duration"]

HOME_JOINTS = PROJECT_CONFIG["robot"]["home_joints"]
THUMB_ONLY = PROJECT_CONFIG["planning"]["thumb_only"]


def pose(d):
    p = Pose()
    p.position.x, p.position.y, p.position.z = d["x"], d["y"], d["z"]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = d["qx"], d["qy"], d["qz"], d["qw"]
    return p


def plan(group):
    p = group.plan()
    return p[1] if isinstance(p, tuple) else p


def save_traj(traj, label, group_name):
    ensure_recording()
    base = ARM_TRAJ_DIR if group_name == "arm" else HAND_TRAJ_DIR
    os.makedirs(base, exist_ok=True)
    name = re.sub(r'/\d+', '', label).replace("/", "_").replace(" ", "_").replace(":", "")
    path = os.path.join(base, name + ".pkl")
    with open(path, "wb") as f:
        pickle.dump(traj, f)
    rospy.loginfo("[%s] saved %s", group_name, path)


def save_joint_delta(delta, label):
    """Save only the hand joints changed by the iterated solution, before planning."""
    ensure_recording()
    os.makedirs(HAND_TRAJ_DIR, exist_ok=True)
    name = re.sub(r'/\d+', '', label).replace("/", "_").replace(" ", "_").replace(":", "")
    path = os.path.join(HAND_TRAJ_DIR, name + ".pkl")
    with open(path, "wb") as f:
        pickle.dump(delta, f)
    rospy.loginfo("[hand] saved iterated joint delta %s", path)


def save_hand_target_joints(joints, label):
    """Save the target hand joint map before planning."""
    ensure_recording()
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

    acm = hand_object_matrix(acm, links, PROJECT_CONFIG, objects)
    diff = PlanningScene(is_diff=True)
    diff.allowed_collision_matrix = acm
    apply_scene(ApplyPlanningSceneRequest(scene=diff))
    rospy.logwarn("Relaxed environment checks for %d objects; retained configured hand-object rule", len(objects))

    def restore():
        diff = PlanningScene(is_diff=True)
        diff.allowed_collision_matrix = original
        apply_scene(ApplyPlanningSceneRequest(scene=diff))

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
    run = start_recording()
    successful = False
    try:
        execute_task()
        successful = True
    finally:
        try:
            finish_run(run, successful)
        finally:
            moveit_commander.roscpp_shutdown()


def execute_task():
    robot = moveit_commander.RobotCommander()
    restore = configure_hand_object_contacts(robot, PROJECT_CONFIG)
    try:
        execute_stages(robot)
    finally:
        restore()


def execute_stages(robot):
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

    execute_pose(arm, ARM_POSE_STAGE5_PRELIFT, "Stage 5a/5: Move arm to additional offset pose")

    restore = allow_robot_world_collisions(robot) if STAGE4_IGNORE_ENV else (lambda: None)
    try:
        execute_cartesian_z_lift(arm, "Stage 5/5: Straight-line Z lift")
        execute_joint_plan(arm, robot, HOME_JOINTS, "After Stage 5: Planned joint move to home")
    finally:
        restore()


if __name__ == "__main__":
    main()
