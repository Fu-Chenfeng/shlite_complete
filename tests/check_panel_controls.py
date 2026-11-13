"""GTK smoke check; run with /usr/bin/python3 on a desktop. No ROS/Docker calls."""
import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import Mock

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'app'))
import control_panel
from desktop_session import Session
from task_definition import TASK_VERSION, STAGES, actions
from trajectory_store import create_run, finish_run

with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    saved = []
    for _ in range(2):
        run = create_run(root / 'trajectories', task_version=TASK_VERSION)
        (run / 'task.json').write_text(json.dumps({'version': TASK_VERSION, 'stages': STAGES}))
        for action in actions():
            (run / action['group'] / action['file']).touch()
            if 'target' in action:
                (run / 'hand' / action['target']).touch()
        finish_run(run, True)
        saved.append(run.name)
    (root / 'config.yaml').write_text('trajectories:\n  replay_run: %s\n' % saved[0])
    session = Session('sim', Mock())
    session.active = session.ready = True
    session.replay = Mock()
    def reset():
        session.reset_requested.clear()
        session.ready = True
        session.task_active = False
    session.reset = Mock(side_effect=reset)
    deferred = Mock()
    control_panel.ROOT = root
    control_panel.Session = lambda *args: session
    errors = []
    def window():
        return next(w for w in Gtk.Window.list_toplevels() if w.get_title().startswith('SH Lite'))
    def guarded(callback):
        def call():
            try:
                return callback()
            except Exception as exc:
                errors.append(exc)
                window().destroy()
                return False
        return call
    def inspect():
        panel = window()
        assert panel.replay_choice.get_active_id() == saved[0]
        assert panel.replay_button.get_sensitive()
        session.lock.acquire()
        panel.work(deferred)  # Queue a task behind the lock, then cancel it.
        session.ready = False
        session.task_active = True
        panel.refresh()
        assert panel.reset_button.get_sensitive(), 'Reset disabled during a running task'
        assert panel.stop_button.get_sensitive(), 'Shutdown disabled during a running task'
        panel.reset_button.clicked()
        session.lock.release()
        GLib.timeout_add(200, guarded(check_reset))
        return False
    def check_reset():
        panel = window()
        session.reset.assert_called_once()
        deferred.assert_not_called()
        assert not panel.resetting
        assert panel.reset_button.get_sensitive()
        panel.replay_choice.set_active_id(saved[1])
        panel.check_button.clicked()
        GLib.timeout_add(200, guarded(check_selection))
        return False
    def check_selection():
        session.replay.assert_called_once_with('check', saved[1])
        window().destroy()
        return False
    GLib.timeout_add(200, guarded(inspect))
    control_panel.main('sim')
    if errors:
        raise errors[0]
    print('GTK check passed: Reset stays enabled during tasks; dropdown selection reaches replay checks.')
