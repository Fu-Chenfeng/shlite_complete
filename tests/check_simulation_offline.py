"""Start headless Gazebo and check shared replay in a network-isolated container.

No NUC connection or real UR driver is started. No planned motion is executed.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tempfile
import xmlrpc.client
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))

ROOT = Path(__file__).resolve().parents[1]
os.environ.update(ROS_MASTER_URI="http://127.0.0.1:11311", ROS_IP="127.0.0.1",
                  GAZEBO_MODEL_PATH=str(ROOT / "assets/models"), SESSION_MODE="sim")
os.environ.pop("ROS_HOSTNAME", None)
children = []
logs = []


def spawn(argv, name):
    log = open("/tmp/" + name + ".log", "w+")
    logs.append(log)
    p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(p)


try:
    spawn(["roscore"], "sim-test-core")
    proxy = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
    deadline = time.monotonic() + 20
    while True:
        try:
            proxy.getPid("/sim_test")
            break
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.1)
    import rospy
    from gazebo_msgs.srv import GetWorldProperties
    from sensor_msgs.msg import JointState

    rospy.init_node("simulation_offline_check", disable_signals=True)
    world_file = os.environ.get("CHECK_WORLD", str(ROOT / "assets/worlds/open.world"))
    subprocess.run([sys.executable, str(ROOT / 'app/simulation_models.py'), world_file], check=True)
    if ET.parse(world_file).find("world/plugin[@name='shlite_hand_object_collisions']") is not None:
        subprocess.run(['bash', str(ROOT / 'docker/build-contact-plugin.sh')], check=True)
    spawn(["roslaunch", str(ROOT / "launch/simulation.launch"), "gui:=false", "world:=" + world_file], "sim-test-launch")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=90)
    world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    assert world.success and "rect_object" in world.model_names, world
    rospy.wait_for_message("/joint_states", JointState, timeout=90)
    rospy.wait_for_service("/get_planning_scene", timeout=90)
    subprocess.run([sys.executable, str(ROOT / 'app/sim_contacts.py')], check=True, timeout=30)
    subprocess.run([sys.executable, str(ROOT / "app/wait_ready.py"), "sim", "--timeout", "20"],
                   check=True, timeout=30)
    import moveit_commander
    from project_config import load_config
    from recording_fixture import make_recording
    moveit_commander.roscpp_initialize([])
    check_robot = moveit_commander.RobotCommander()
    check_groups = {name: moveit_commander.MoveGroupCommander('right_' + name) for name in ('arm', 'hand')}
    if load_config()['object'].get('collision_mesh', False):
        from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
        from moveit_msgs.msg import PlanningSceneComponents
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        deadline = time.monotonic() + 20
        while True:
            target_objects = [obj for obj in get_scene(request).scene.world.collision_objects
                              if 'rect_object' in obj.id]
            if target_objects and any(obj.meshes for obj in target_objects):
                break
            if time.monotonic() > deadline:
                raise AssertionError('Imported object mesh did not reach the MoveIt scene: %s' % target_objects)
            time.sleep(0.2)
        print('Imported tofu collision mesh is present in Gazebo and MoveIt.', flush=True)
    with tempfile.TemporaryDirectory(prefix='shlite-five-stage-fixture-') as archive:
        fixture = make_recording(archive, check_robot, check_groups, load_config())
        subprocess.run([sys.executable, str(ROOT / 'app/replay.py'), '--check',
                        '--trajectory-root', str(fixture)], check=True, timeout=30)
    assert rospy.get_param("/use_sim_time") is True
    print("Headless Gazebo, arm/hand controllers, MoveIt and shared replay preflight passed.", flush=True)
    if os.environ.get('CHECK_REPLAY') == '1' and os.environ.get('CHECK_PLANNER') != '1':
        raise ValueError('CHECK_REPLAY requires CHECK_PLANNER=1 to generate a compatible recording')
    if os.environ.get("CHECK_PLANNER") == "1":
        with tempfile.TemporaryDirectory(prefix="shlite-plan-test-") as output:
            env = dict(os.environ, TRAJECTORY_ROOT=output)
            try:
                subprocess.run([sys.executable, str(ROOT / "app/plan.py")], env=env, check=True, timeout=600)
            except subprocess.CalledProcessError:
                from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetStateValidity
                from moveit_msgs.msg import PlanningSceneComponents
                request = GetPlanningSceneRequest()
                request.components.components = PlanningSceneComponents.ROBOT_STATE
                state = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)(request).scene.robot_state
                validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)(robot_state=state, group_name='right_arm')
                print('Planner failure collision pairs:',
                      [(c.contact_body_1, c.contact_body_2) for c in validity.contacts], flush=True)
                raise
            subprocess.run([sys.executable, str(ROOT / "app/replay.py"), "--check",
                            "--trajectory-root", output], env=env, check=True, timeout=30)
            from trajectory_store import resolve_run
            from task_definition import TASK_VERSION
            from verify_five_stage_run import verify_recording
            run = resolve_run(output, task_version=TASK_VERSION)
            verify_recording(run, check_groups, load_config())
            print('Five-stage planning, configured palm lift, arm home and hand zero verified.', flush=True)
            if os.environ.get('CHECK_REPLAY') == '1':
                subprocess.run([sys.executable, str(ROOT / 'app/replay.py'), '--run',
                                '--trajectory-root', str(run)], check=True, timeout=600)
                verify_recording(run, check_groups, load_config())
                print('All five saved stages replayed successfully.', flush=True)
    moveit_commander.roscpp_shutdown()
except BaseException:
    for log in logs:
        log.flush()
        log.seek(0)
        print(log.name, log.read()[-10000:], file=sys.stderr)
    raise
finally:
    if "rospy" in sys.modules:
        sys.modules["rospy"].signal_shutdown("Simulation test complete")
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
