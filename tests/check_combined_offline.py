"""Headless combined-model check. Run ONLY inside a --network none container.

Starts an isolated ROS master and publishes synthetic joint/controller states.
No hardware driver, trajectory controller or trajectory execution is started.
"""
import os
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import tempfile
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
os.environ["ROS_IP"] = "127.0.0.1"
os.environ.pop("ROS_HOSTNAME", None)
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "app"))
children = []
logs = []


def spawn(argv, name):
    log = open("/tmp/" + name + ".log", "w+")
    logs.append(log)
    process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(process)


try:
    spawn(["roscore"], "test-core")
    master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
    deadline = time.monotonic() + 20
    while True:
        try:
            master.getPid("/offline_check")
            break
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.1)

    import rospy
    from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryResult
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import PlanningScene
    import moveit_commander
    import tf2_ros
    import actionlib

    rospy.init_node("offline_combined_test", disable_signals=True)
    # Let MoveIt discover controller endpoints without the 34-second connection
    # timeout. These stubs reject every goal; the check must send none.
    unexpected_goals = []
    endpoints = []
    def reject(goal):
        unexpected_goals.append(goal)
        for endpoint in endpoints:
            if endpoint.is_active():
                endpoint.set_aborted(FollowJointTrajectoryResult(error_code=-1))
    for prefix in ("ra", "rh"):
        endpoints.append(actionlib.SimpleActionServer(
            "/%s_trajectory_controller/follow_joint_trajectory" % prefix,
            FollowJointTrajectoryAction, execute_cb=reject, auto_start=False,
        ))
    for endpoint in endpoints:
        endpoint.start()
    spawn(["roslaunch", str(root / "launch/real.launch"), "gui:=false"], "test-combined")
    deadline = time.monotonic() + 20
    while not rospy.has_param("/robot_description"):
        if time.monotonic() > deadline:
            raise RuntimeError("Combined description did not load")
        time.sleep(0.1)
    robot = ET.fromstring(rospy.get_param("/robot_description"))
    names = [j.attrib["name"] for j in robot.findall("joint") if j.attrib["type"] != "fixed" and j.find("mimic") is None]
    joint_publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
    arm_publisher = rospy.Publisher("/ra_trajectory_controller/state", JointTrajectoryControllerState, queue_size=10)
    hand_publisher = rospy.Publisher("/rh_trajectory_controller/state", JointTrajectoryControllerState, queue_size=10)

    def publish():
        while not rospy.is_shutdown():
            joints = JointState(name=names, position=[0.0] * len(names))
            # Trigger state-update publication instead of keeping every joint
            # identical to MoveIt's initial all-zero state.
            joints.position[0] = 0.001 * math.sin(time.monotonic())
            joints.header.stamp = rospy.Time.now()
            joint_publisher.publish(joints)
            for prefix, publisher in (("ra_", arm_publisher), ("rh_", hand_publisher)):
                message = JointTrajectoryControllerState()
                message.header.stamp = joints.header.stamp
                message.joint_names = [name for name in names if name.startswith(prefix)]
                for point in (message.actual, message.desired, message.error):
                    point.positions = [0.0] * len(message.joint_names)
                publisher.publish(message)
            time.sleep(0.05)

    threading.Thread(target=publish, daemon=True).start()
    rospy.wait_for_service("/get_planning_scene", timeout=60)
    rospy.wait_for_message("/move_group/monitored_planning_scene", PlanningScene, timeout=10)
    moveit_commander.roscpp_initialize(sys.argv)
    groups = moveit_commander.RobotCommander().get_group_names()
    assert "right_arm" in groups and "right_hand" in groups, groups
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    buffer.lookup_transform("world", "rh_palm", rospy.Time(0), rospy.Duration(10))
    subprocess.run([sys.executable, str(root / "app/add_cell_obstacles.py")], check=True, timeout=30)
    from project_config import load_config
    config = load_config()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    objects = scene.get_objects()
    for definition in config["environment"]:
        if definition["enabled"]:
            obj = objects[definition["name"]]
            assert list(obj.primitives[0].dimensions) == definition["size"], obj
            # MoveIt versions store the global offset on either the object or primitive.
            position = (obj.pose if hasattr(obj, 'pose') else obj.primitive_poses[0]).position
            assert [position.x, position.y, position.z] == definition["pose"][:3], obj
    print("Configured environment boxes loaded into combined MoveIt scene.", flush=True)
    import copy
    from sim_contacts import configure_hand_object_contacts
    from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
    from moveit_msgs.msg import PlanningSceneComponents
    request = GetPlanningSceneRequest()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    robot = moveit_commander.RobotCommander()
    for enabled in (False, True):
        test_config = copy.deepcopy(config)
        test_config['object']['hand_collision'] = enabled
        restore = configure_hand_object_contacts(robot, test_config)
        try:
            matrix = get_scene(request).scene.allowed_collision_matrix
            def allowed_pair(first, second):
                if first in matrix.entry_names and second in matrix.entry_names:
                    return matrix.entry_values[matrix.entry_names.index(first)].enabled[matrix.entry_names.index(second)]
                # MoveIt omits entries identical to the default (checked/false).
                defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
                values = [defaults[name] for name in (first, second) if name in defaults]
                return all(values) if values else False
            for name in robot.get_link_names():
                if name.startswith('rh_'):
                    for object_name in ('rect_object', 'rect_object__link'):
                        assert allowed_pair(object_name, name) == (not enabled), (object_name, name)
        finally:
            restore()
    print('Hand-object MoveIt collision switch checked in both directions.', flush=True)
    print("Combined MoveIt started; right_arm and right_hand loaded; world-to-palm TF connected.", flush=True)
    from recording_fixture import make_recording
    with tempfile.TemporaryDirectory(prefix='shlite-five-stage-check-') as archive:
        replay_groups = {name: moveit_commander.MoveGroupCommander('right_' + name) for name in ('arm', 'hand')}
        recording = make_recording(archive, robot, replay_groups, config)
        subprocess.run([
            sys.executable, str(root / 'app/replay.py'), '--check',
            '--trajectory-root', str(recording),
        ], check=True, timeout=30)
        from plan import hand_grasp_flange_pose, verify_lift_endpoint
        try:
            verify_lift_endpoint(replay_groups['arm'], robot,
                                 hand_grasp_flange_pose(replay_groups['arm'], config['grasp']))
            print('Configured lift endpoint is reachable.', flush=True)
        except RuntimeError as exc:
            print('Configured lift reachability report:', exc, flush=True)
    assert not unexpected_goals, "Read-only check attempted a motion"
    moveit_commander.roscpp_shutdown()
except BaseException:
    for log in logs:
        log.flush()
        log.seek(0)
        print(log.name, log.read()[-7000:], file=sys.stderr)
    raise
finally:
    if "rospy" in sys.modules:
        sys.modules["rospy"].signal_shutdown("Offline test finished")
    for process in reversed(children):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
