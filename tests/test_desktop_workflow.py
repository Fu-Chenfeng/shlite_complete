"""Host-only configuration and desktop orchestration checks. Never starts ROS."""
import copy
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import json
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
import project_config as settings
from desktop_session import Cancelled, Session
from desktop_shortcuts import script_entries

loader = importlib.machinery.SourceFileLoader('desktop_test_robotctl', str(ROOT / 'robotctl'))
spec = importlib.util.spec_from_loader(loader.name, loader)
ctl = importlib.util.module_from_spec(spec)
loader.exec_module(ctl)


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        shutil.copytree(ROOT / 'assets', self.root / 'assets')
        shutil.copy(ROOT / 'config.yaml', self.root / 'config.yaml')
        self.config = settings.load_config(self.root)

    def write(self):
        (self.root / 'config.yaml').write_text(yaml.safe_dump(self.config))

    def test_object_dimensions_pose_color_and_mass_reach_generated_scene(self):
        obj = self.config['object']
        obj.update(visual_size=[0.1, 0.2, 0.3], collision_size=[0.09, 0.19, 0.29],
                   pose=[0.5, 0.1, 0.2, 0, 0, 0.5], color=[0, 1, 0, 1], mass=0.4)
        original = (self.root / obj['model']).read_bytes()
        output = settings.prepare_simulation(self.config, root=self.root)
        model = ET.parse(output).find("world/model[@name='rect_object']")
        self.assertEqual(model.findtext('pose'), '0.5 0.1 0.2 0 0 0.5')
        self.assertEqual(model.findtext('link/visual/geometry/box/size'), '0.1 0.2 0.3')
        self.assertEqual(model.findtext('link/collision/geometry/box/size'), '0.09 0.19 0.29')
        self.assertEqual(model.findtext('link/visual/material/diffuse'), '0 1 0 1')
        self.assertEqual(model.findtext('link/inertial/mass'), '0.4')
        self.assertEqual((self.root / obj['model']).read_bytes(), original)
        restored = ET.parse(self.root / '.runtime/models/rect_object/model.sdf')
        self.assertEqual(restored.findtext('model/link/visual/geometry/box/size'), '0.1 0.2 0.3')

    def test_mesh_uses_container_path_and_separate_collision_box(self):
        self.config['object']['mesh'] = 'assets/models/tofu_real.stl'
        self.config['object']['mesh_scale'] = [0.001] * 3
        model = settings.object_model(self.config, self.root)
        self.assertEqual(model.findtext('link/visual/geometry/mesh/uri'),
                         'file:///project/assets/models/tofu_real.stl')
        self.assertEqual(model.findtext('link/visual/geometry/mesh/scale'), '0.001 0.001 0.001')
        self.assertIsNotNone(model.find('link/collision/geometry/box'))

    def test_disabled_object_is_removed(self):
        self.config['object']['enabled'] = False
        world = ET.parse(settings.prepare_simulation(self.config, root=self.root))
        self.assertIsNone(world.find("world/model[@name='rect_object']"))
        self.assertIsNone(world.find("world/include[name='rect_object']"))

    def test_hand_collision_switch_adds_only_the_targeted_physics_filter(self):
        for enabled in (True, False):
            self.config['object']['hand_collision'] = enabled
            world = ET.parse(settings.prepare_simulation(self.config, root=self.root))
            plugin = world.find("world/plugin[@name='shlite_hand_object_collisions']")
            self.assertEqual(plugin is None, enabled)
            obj = world.find("world/model[@name='rect_object']")
            self.assertEqual(obj.findtext('link/collision/geometry/box/size'), '0.031 0.087 0.072')
            self.assertIsNotNone(world.find("world/model[@name='ground_plane_user']/link/collision"))

    def test_cell_and_open_worlds_keep_their_environment_selection(self):
        self.config['environment'][0]['size'] = [1, 2, 0.01]
        self.config['environment'][1]['enabled'] = False
        for filename, expected in [('open.world', 2), ('cell.world', 5)]:
            world = ET.parse(settings.prepare_simulation(self.config, 'assets/worlds/' + filename, self.root))
            self.assertEqual(len(world.findall('world/model')), expected)
            self.assertEqual(world.findtext("world/model[@name='ground_plane_user']/link/visual/geometry/box/size"),
                             '1 2 0.01')
            self.assertIsNone(world.find("world/model[@name='cell_plane_user']"))

    def test_invalid_values_fail_with_a_config_error(self):
        for change in [lambda c: c['object'].update(collision_size=[-1, 1, 1]),
                       lambda c: c['object'].update(pose=[0, 0]),
                       lambda c: c['object'].update(enabled='false'),
                       lambda c: c['object'].update(hand_collision='false'),
                       lambda c: c['network'].update(ros_port=70000),
                       lambda c: c['planning'].update(stage5_duration=float('nan'))]:
            self.config = copy.deepcopy(settings.load_config(ROOT))
            change(self.config)
            self.write()
            with self.assertRaisesRegex(ValueError, 'Invalid config.yaml'):
                settings.load_config(self.root)

    def test_paths_must_be_inside_project_and_can_use_container_prefix(self):
        self.assertEqual(settings.project_path('/project/assets/models/tofu_real.stl', self.root),
                         self.root / 'assets/models/tofu_real.stl')
        with self.assertRaises(ValueError):
            settings.project_path('/etc/passwd', self.root)

    def test_network_yaml_overrides_legacy_dotenv_and_env_overrides_yaml(self):
        (self.root / '.env').write_text('PC_IP=old-address\nSSH_KEY=old-key\n')
        with patch.object(ctl, 'ROOT', self.root), patch.object(ctl, 'load_config', return_value=self.config), patch.dict(os.environ, {}, clear=True):
            self.assertEqual(ctl.configuration()['PC_IP'], self.config['network']['pc_ip'])
            with patch.dict(os.environ, {'PC_IP': 'test-host'}):
                self.assertEqual(ctl.configuration()['PC_IP'], 'test-host')

    def test_generated_world_and_robot_settings_are_forwarded_without_duplicate_args(self):
        cfg = ctl.configuration()
        with patch.object(ctl, 'ROOT', self.root), patch.object(ctl, 'ensure_no_master'), patch.object(ctl, 'load_config', return_value=self.config), patch.object(ctl, 'prepare_simulation', return_value=self.root / '.runtime/simulation.world'), patch.object(ctl, 'ros_command') as execute:
            ctl.mode_command(cfg, 'sim', 'start', ['gui:=false', 'world:=/project/assets/worlds/cell.world'])
            argv = execute.call_args.args[2]
            self.assertEqual([arg for arg in argv if arg.startswith('gui:=')], ['gui:=false'])
            self.assertIn('world:=/project/.runtime/simulation.world', argv)
            self.assertIn('initial_z:=0.012', argv)

    def test_project_launchers_work_after_moving_without_installation(self):
        moved = self.root / 'moved project #1'
        moved.mkdir()
        # A harmless stand-in records the requested panel instead of opening it.
        (moved / 'robotctl').write_text(
            'import json,sys\nfrom pathlib import Path\n'
            'Path(__file__).with_name("invocation.json").write_text(json.dumps(sys.argv[1:]))\n')
        for filename, entry in script_entries():
            script = moved / filename
            script.write_text(entry)
            script.chmod(0o755)
            self.assertNotIn(str(ROOT), entry)
            mode = 'sim' if filename == 'Simulation.sh' else 'real'
            subprocess.run([str(script)], cwd='/tmp', check=True)
            self.assertEqual(json.loads((moved / 'invocation.json').read_text()), ['panel', mode])

    def test_shutdown_uses_captured_network_without_loading_edited_config(self):
        (self.root / '.env').write_text('')
        (self.root / '.runtime').mkdir()
        captured = {'PC_IP': 'original-host', 'NUC_IP': 'original-nuc'}
        (self.root / '.runtime/real-environment.json').write_text(json.dumps(captured))
        with patch.object(ctl, 'ROOT', self.root), patch.object(ctl, 'configuration', side_effect=ValueError('edited config invalid')), patch.object(ctl, 'mode_command') as command, patch.object(sys, 'argv', ['robotctl', 'real', 'shutdown']):
            ctl.main()
            command.assert_called_once_with(captured, 'real', 'shutdown', [])

    def test_readiness_waits_for_ros_core_before_running_container_probe(self):
        cfg = ctl.configuration()
        with patch.object(ctl, 'master', side_effect=[ctl.UserError('starting'), Mock()]) as master, patch.object(ctl.time, 'sleep'), patch.object(ctl, 'ros_command') as execute:
            ctl.mode_command(cfg, 'sim', 'wait', ['sim'])
            self.assertEqual(master.call_count, 2)
            execute.assert_called_once_with(cfg, 'sim', ['python3', '/project/app/wait_ready.py', 'sim'])


class DesktopSessionTest(unittest.TestCase):
    def session(self, mode):
        session = Session(mode, Mock())
        session.run = Mock()
        session.launch = Mock()
        return session

    def test_simulation_start_never_launches_hardware_or_motion(self):
        session = self.session('sim')
        session.start()
        self.assertEqual(session.launch.call_args_list, [unittest.mock.call('start')])
        self.assertTrue(session.ready)
        self.assertEqual(session.run.call_args_list, [
            unittest.mock.call('init'), unittest.mock.call('validate-config'), unittest.mock.call('gui'),
            unittest.mock.call('sim', 'preflight'), unittest.mock.call('sim', 'up'),
            unittest.mock.call('sim', 'wait', 'sim'), unittest.mock.call('sim', 'collisions')])

    def test_real_start_waits_for_each_dependency_without_replaying(self):
        session = self.session('real')
        events = []
        session.run.side_effect = lambda *args, **kwargs: events.append(args)
        session.launch.side_effect = lambda name: events.append(('launch', name))
        session.start()
        self.assertEqual(events[5:], [
            ('launch', 'core'), ('real', 'wait-core'), ('launch', 'nuc'), ('real', 'wait', 'hand'),
            ('launch', 'ur5'), ('real', 'wait', 'ur5'), ('launch', 'adapter'), ('launch', 'moveit'),
            ('real', 'wait', 'real'), ('real', 'obstacles'), ('real', 'collisions')])
        self.assertTrue(session.ready)

    def test_hand_failure_prevents_ur5_start_and_leaves_shutdown_available(self):
        session = self.session('real')
        def execute(*args, **kwargs):
            if args == ('real', 'wait', 'hand'):
                raise RuntimeError('hand unavailable')
        session.run.side_effect = execute
        with self.assertRaisesRegex(RuntimeError, 'hand unavailable'):
            session.start()
        self.assertEqual(session.launch.call_args_list, [unittest.mock.call('core'), unittest.mock.call('nuc')])
        self.assertTrue(session.active)
        self.assertFalse(session.ready)

    def test_failed_preflight_does_not_mark_session_active(self):
        session = self.session('real')
        session.run.side_effect = RuntimeError('docker unavailable')
        with self.assertRaises(RuntimeError):
            session.start()
        self.assertFalse(session.active)
        session.launch.assert_not_called()

    def test_partial_container_start_keeps_shutdown_available(self):
        session = self.session('real')
        def execute(*args, **kwargs):
            if args == ('real', 'up'):
                raise RuntimeError('container start failed')
        session.run.side_effect = execute
        with self.assertRaises(RuntimeError):
            session.start()
        self.assertTrue(session.active)
        session.launch.assert_not_called()

    def test_failed_calibration_keeps_processes_and_session_for_retry(self):
        session = self.session('real')
        session.active = True
        process = Mock()
        session.children = [('nuc', process)]
        session.run.side_effect = RuntimeError('calibration failed')
        with self.assertRaises(RuntimeError):
            session.shutdown()
        session.run.assert_called_once_with('real', 'shutdown', timeout=240, stopping=True)
        process.wait.assert_not_called()
        self.assertTrue(session.active)

    def test_shutdown_before_hand_start_stops_only_workstation(self):
        session = self.session('real')
        session.active = True
        session.children = [('core', Mock())]
        session.shutdown()
        session.run.assert_called_once_with('real', 'stop-idle', timeout=240, stopping=True)
        self.assertFalse(session.active)

    def test_cancelled_start_never_launches_a_process(self):
        session = Session('real', Mock())
        session.cancel.set()
        with patch.object(session, 'spawn') as spawn:
            with self.assertRaises(Cancelled):
                session.launch('nuc')
            spawn.assert_not_called()


if __name__ == '__main__':
    unittest.main()
