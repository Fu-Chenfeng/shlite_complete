"""Network-isolated Gazebo test: only hand-object contacts are disabled.

Drops simple test boxes; launches no robot or robot controller.
"""
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1')
os.environ.pop('ROS_HOSTNAME', None)
children = []


def box(world, name, link_name, xyz, size, static=False):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = str(static).lower()
    ET.SubElement(model, 'pose').text = ' '.join(str(v) for v in [*xyz, 0, 0, 0])
    link = ET.SubElement(model, 'link', name=link_name)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '1'
    inertia = ET.SubElement(inertial, 'inertia')
    for key in ('ixx', 'iyy', 'izz'):
        ET.SubElement(inertia, key).text = '0.01'
    geometry = ET.SubElement(ET.SubElement(link, 'collision', name=link_name + '_collision'), 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))


with tempfile.TemporaryDirectory(prefix='shlite-contact-check-') as folder:
    log = open(Path(folder) / 'gazebo.log', 'w+')
    try:
        subprocess.run(['bash', str(ROOT / 'docker/build-contact-plugin.sh')], check=True)
        sdf = ET.Element('sdf', version='1.6')
        world = ET.SubElement(sdf, 'world', name='default')
        ET.SubElement(world, 'gravity').text = '0 0 -9.81'
        physics = ET.SubElement(world, 'physics', name='default', type='ode')
        ET.SubElement(physics, 'max_step_size').text = '0.001'
        plugin = ET.SubElement(world, 'plugin', name='shlite_hand_object_collisions',
                               filename='/tmp/shlite-contact-filter/libhand_object_collisions.so')
        ET.SubElement(plugin, 'object_name').text = 'rect_object'
        box(world, 'floor', 'floor', [0, 0, -0.05], [20, 20, 0.1], True)
        box(world, 'rect_object', 'link', [0, 0, 0.5], [1, 1, 0.2], True)
        box(world, 'reference_object', 'link', [3, 0, 0.5], [1, 1, 0.2], True)
        expected = {
            'hand_off': ('rh_probe', [0, 0.2, 1.4], 0.05),
            'arm': ('ra_probe', [0.3, 0, 1.4], 0.65),
            'other': ('other_probe', [-0.3, 0, 1.4], 0.65),
            'hand_floor': ('rh_floor_probe', [1.5, 0, 1.4], 0.05),
            'hand_on': ('rh_enabled_probe', [3, 0, 1.4], 0.65),
        }
        for name, (link, xyz, _) in expected.items():
            box(world, name, link, xyz, [0.1, 0.1, 0.1])
        path = Path(folder) / 'contacts.world'
        ET.ElementTree(sdf).write(path)
        children.append(subprocess.Popen(['roscore'], stdout=log, stderr=log, start_new_session=True))
        proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 20
        while True:
            try:
                proxy.getPid('/contact_check')
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.1)
        import rospy
        from gazebo_msgs.srv import GetModelState
        rospy.init_node('hand_contact_check', disable_signals=True)
        children.append(subprocess.Popen(
            ['roslaunch', 'gazebo_ros', 'empty_world.launch', 'world_name:=' + str(path),
             'gui:=false', 'paused:=false'], stdout=log, stderr=log, start_new_session=True))
        rospy.wait_for_service('/gazebo/get_model_state', timeout=90)
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        deadline = time.monotonic() + 30
        while True:
            positions = {name: get_state(name, 'world').pose.position.z for name in expected}
            if all(abs(positions[name] - goal) < 0.015 for name, (_, _, goal) in expected.items()):
                break
            if time.monotonic() > deadline:
                raise AssertionError('Unexpected resting heights: %r' % positions)
            time.sleep(0.2)
        print('Hand-object contacts disabled; arm-object, other-object, hand-floor and enabled-object contacts preserved.')
        print('Resting heights:', positions)
    except BaseException:
        log.flush()
        log.seek(0)
        print(log.read()[-12000:])
        raise
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
