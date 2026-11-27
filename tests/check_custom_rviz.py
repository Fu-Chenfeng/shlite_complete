"""Load an isolated RViz and capture its editable goal; never execute motion."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import rospy
from moveit_msgs.msg import RobotState
import yaml

from custom_trajectory import RVIZ_GOAL_TOPIC

rospy.init_node('check_custom_rviz_bridge', anonymous=True)
config = {'Visualization Manager': {
    'Global Options': {'Fixed Frame': 'world'},
    'Displays': [{'Class': 'shlite_custom_rviz/MotionPlanning', 'Name': 'MotionPlanning',
                  'Enabled': True, 'Robot Description': 'robot_description',
                  'Planning Scene Topic': '/move_group/monitored_planning_scene',
                  'Planning Request': {'Planning Group': 'right_arm'}}]}}
with tempfile.TemporaryDirectory(prefix='custom-rviz-') as folder:
    path = Path(folder) / 'check.rviz'
    path.write_text(yaml.safe_dump(config))
    with (Path(folder) / 'rviz.log').open('w') as log:
        process = subprocess.Popen(['rviz', '-d', str(path)], stdout=log, stderr=subprocess.STDOUT)
        try:
            message = rospy.wait_for_message(RVIZ_GOAL_TOPIC, RobotState, timeout=20)
            names = set(message.joint_state.name)
            assert 'ra_shoulder_pan_joint' in names and 'rh_THJ5' in names
            capture = subprocess.run([sys.executable, '/project/app/custom_motion.py', 'capture', '--source', 'rviz'],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
            if capture.returncode:
                raise RuntimeError(capture.stdout)
            payload = capture.stdout.partition('CUSTOM_STATE_JSON=')[2]
            result = json.JSONDecoder().raw_decode(payload)[0]
            assert len(result['arm']) == 6 and len(result['hand']) == 16
            print('RViz bridge passed: captured six arm and sixteen hand goal angles.', flush=True)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
