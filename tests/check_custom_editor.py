"""GTK interaction smoke check with fake robot I/O; run under Xvfb."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from unittest.mock import Mock

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
import custom_trajectory_editor as ui


def state(a=.1, h=.2, source='current'):
    return {'source': source, 'arm': {'a': a}, 'hand': {'h': h},
            'bounds': {'a': [-2, 2], 'h': [-1, 1]}}


class Panel(Gtk.Window):
    def __init__(self):
        super().__init__(title='Test panel')
        self.session = Mock(ready=True)
        self.session.capture_state.side_effect = lambda source: state(1.1, .5, source) if source == 'rviz' else state()
        self.busy = self.stopping = self.resetting = False
        self.refresh_runs = Mock()
        self.session.terminal_task.side_effect = self.terminal
        self.document = None

    def terminal(self, command, *args):
        assert command == 'custom-plan'
        path = ui.ROOT / args[1].removeprefix('/project/')
        self.document = json.loads(path.read_text())
        assert '--execute' in args

    def work(self, action, on_success=None):
        result = action()
        if on_success:
            on_success(result)


def children(widget):
    result = [widget]
    if isinstance(widget, Gtk.Container):
        for child in widget.get_children():
            result.extend(children(child))
    return result


with tempfile.TemporaryDirectory() as folder:
    ui.ROOT = Path(folder)
    panel = Panel()
    editor = ui.create_editor(panel, Gtk, GLib)
    assert editor.sequence.initial['arm'] == {'a': .1}
    assert not editor.plan_button.get_sensitive()
    editor.add_button.clicked()
    assert len(editor.sequence.tasks) == 1
    assert editor.plan_button.get_sensitive()
    editor.capture_button.clicked()
    assert len(editor.sequence.recordings) == 1
    assert 'State 1' in editor.details.get_text()
    errors = []
    def edit_dialog():
        try:
            dialog = next(w for w in Gtk.Window.list_toplevels() if isinstance(w, Gtk.Dialog))
            widgets = children(dialog)
            grids = [w for w in widgets if isinstance(w, Gtk.Grid)]
            arm = next(g for g in grids if g.get_child_at(0, 0).get_text() == 'a').get_child_at(1, 0)
            hand = next(g for g in grids if g.get_child_at(0, 0).get_text() == 'h').get_child_at(1, 0)
            arm.set_text('30')
            read_hand = next(w for w in widgets if isinstance(w, Gtk.Button) and w.get_label() == 'Read hand goal')
            read_hand.clicked()
            assert arm.get_text() == '30', 'Reading hand overwrote arm goal'
            assert abs(math.radians(float(hand.get_text())) - .5) < 1e-9
            read_arm = next(w for w in widgets if isinstance(w, Gtk.Button) and w.get_label() == 'Read arm goal')
            read_arm.clicked()
            assert abs(math.radians(float(arm.get_text())) - 1.1) < 1e-9
            assert abs(math.radians(float(hand.get_text())) - .5) < 1e-9
            arm.set_text('30')
            dialog.response(Gtk.ResponseType.OK)
        except Exception as exc:
            errors.append(exc)
            dialog.response(Gtk.ResponseType.CANCEL)
        return False
    GLib.idle_add(edit_dialog)
    editor.task_view.row_activated(Gtk.TreePath.new_from_indices([0]), editor.task_view.get_column(0))
    if errors:
        raise errors[0]
    assert abs(editor.sequence.tasks[0]['arm']['a'] - math.pi/6) < 1e-9
    assert abs(editor.sequence.tasks[0]['hand']['h'] - .5) < 1e-9
    editor.add_button.clicked()
    assert editor.sequence.tasks[1]['arm'] == editor.sequence.tasks[0]['arm']
    editor.sequence.tasks[1]['arm']['a'] = -.2
    assert editor.sequence.tasks[0]['arm']['a'] > 0
    editor.remove_button.clicked()
    assert len(editor.sequence.tasks) == 1
    panel.busy = True
    editor.refresh()
    assert not editor.capture_button.get_sensitive()
    assert not editor.plan_button.get_sensitive()
    panel.busy = False
    editor.refresh()
    editor.plan_button.clicked()
    assert panel.document['name'].startswith('custom_')
    assert len(panel.document['tasks']) == 1
    assert len(panel.document['recordings']) == 1
    assert not list((ui.ROOT / '.runtime').glob('custom-*.json'))
    editor.destroy()
    panel.destroy()
print('GTK custom editor passed: capture, edit, separate goal imports, inheritance, delete, save and busy controls.')
