"""Plan/save/check two custom tasks against running simulation; never execute."""
from pathlib import Path
import sys
import subprocess
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import moveit_commander
import rospy
from custom_motion import capture, configure_groups, load_recording, plan_sequence, save_sequence
from custom_trajectory import Sequence, TASK_VERSION
from project_config import load_config
from trajectory_store import create_run, finish_run, replay_run_choices

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('check_custom_planning', anonymous=True)
try:
    robot = moveit_commander.RobotCommander()
    groups = {name: moveit_commander.MoveGroupCommander('right_' + name) for name in ('arm', 'hand')}
    initial = capture(robot, groups, 'current')
    sequence = Sequence(initial)
    sequence.add()
    sequence.add()['arm']['ra_shoulder_pan_joint'] += .02
    document = sequence.document('custom_integration_check')
    config = load_config()
    config['planning']['planning_time'] = 5
    configure_groups(groups, config['planning'])
    planned = plan_sequence(robot, groups, document)
    with tempfile.TemporaryDirectory(prefix='custom-integration-') as folder:
        run = create_run(folder, task_version=TASK_VERSION, name=document['name'])
        save_sequence(run, document, config, planned)
        finish_run(run, True)
        _, loaded = load_recording(run, robot, groups)
        assert len(loaded) == 4
        assert replay_run_choices(folder)[0]['replayable']
        subprocess.run([sys.executable, '/project/app/replay.py', '--check', '--trajectory-root', str(run)],
                       check=True, timeout=20)
    print('Live MoveIt check passed: two tasks planned, saved and validated; no motion executed.', flush=True)
finally:
    moveit_commander.roscpp_shutdown()
