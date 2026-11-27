"""Check panel-only world selection in both GTK panels without ROS or hardware."""
from pathlib import Path
import shutil
import sys
import tempfile
from unittest.mock import Mock

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gtk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
import control_panel
from desktop_session import Session

with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    shutil.copytree(ROOT / 'assets/worlds', root / 'assets/worlds')
    config = root / 'config.yaml'
    original = '# World choice must stay outside this file.\ntrajectories:\n  replay_run: latest\n'
    config.write_text(original)
    control_panel.ROOT = root
    for mode in ('sim', 'real'):
        session = Session(mode, Mock())
        def start():
            session.active = session.ready = True
        session.start = Mock(side_effect=start)
        control_panel.Session = lambda *args: session
        errors = []
        def panel():
            return next(w for w in Gtk.Window.list_toplevels() if (w.get_title() or '').startswith('SH Lite'))
        def guarded(callback):
            def run():
                try:
                    return callback()
                except Exception as exc:
                    errors.append(exc)
                    panel().destroy()
                    return False
            return run
        def inspect():
            window = panel()
            assert window.world_choice.get_sensitive()
            assert window.world_choice.get_active_id() == 'assets/worlds/open.world'
            assert [row[0] for row in window.world_choice.get_model()] == [
                'tofu', 'tofu no collision', 'tofu cell', 'no object']
            assert window.world_choice.set_active_id('assets/worlds/no_object.world')
            assert session.world == 'assets/worlds/no_object.world'
            assert config.read_text() == original
            window.start_button.clicked()
            assert not window.world_choice.get_sensitive()
            GLib.timeout_add(200, guarded(check_started))
            return False
        def check_started():
            window = panel()
            session.start.assert_called_once_with()
            assert not window.world_choice.get_sensitive()
            assert session.world == 'assets/worlds/no_object.world'
            assert config.read_text() == original
            session.active = session.ready = False
            window.refresh()
            assert window.world_choice.get_sensitive()
            window.destroy()
            return False
        GLib.timeout_add(100, guarded(inspect))
        control_panel.main(mode)
        if errors:
            raise errors[0]
print('GTK worlds passed: both panels select a session world without editing config.yaml.')
