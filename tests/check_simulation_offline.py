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
    subprocess.run([sys.executable, str(ROOT / "app/replay.py"), "--check",
                    "--trajectory-root", str(ROOT / "trajectories")], check=True, timeout=30)
    assert rospy.get_param("/use_sim_time") is True
    print("Headless Gazebo, arm/hand controllers, MoveIt and shared replay preflight passed.", flush=True)
    if os.environ.get("CHECK_REPLAY") == "1":
        subprocess.run([sys.executable, str(ROOT / "app/replay.py"), "--run",
                        "--trajectory-root", str(ROOT / "trajectories")], check=True, timeout=300)
        print("Bundled saved task replayed all stages in simulation.", flush=True)
    if os.environ.get("CHECK_PLANNER") == "1":
        with tempfile.TemporaryDirectory(prefix="shlite-plan-test-") as output:
            env = dict(os.environ, TRAJECTORY_ROOT=output)
            try:
                subprocess.run([sys.executable, str(ROOT / "app/plan.py")], env=env, check=True, timeout=300)
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
            print("Planning, simulation execution and saving all stages passed; fresh files match shared replay.", flush=True)
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
