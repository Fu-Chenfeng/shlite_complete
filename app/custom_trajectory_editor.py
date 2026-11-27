"""GTK custom trajectory editor; ROS work runs through the desktop session."""
import copy
import json
import math
import tempfile
from pathlib import Path

from custom_trajectory import Sequence, default_name
from desktop_session import ROOT


def create_editor(panel, Gtk, GLib):
    class Editor(Gtk.Window):
        def __init__(self):
            super().__init__(title='Custom trajectory', transient_for=panel)
            self.set_destroy_with_parent(True)
            self.set_default_size(980, 640)
            self.set_border_width(16)
            self.sequence = None
            self.bounds = {}
            self.closed = False
            self.connect('destroy', lambda *_: setattr(self, 'closed', True))
            self.connect('delete-event', self.hide_on_delete)
            outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
            self.add(outer)
            naming = Gtk.Box(spacing=10)
            naming.pack_start(Gtk.Label(label='Trajectory name:'), False, False, 0)
            self.name = Gtk.Entry(text=default_name())
            naming.pack_start(self.name, True, True, 0)
            outer.pack_start(naming, False, False, 0)
            columns = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
            columns.set_position(440)
            outer.pack_start(columns, True, True, 0)
            left, right = [Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10) for _ in range(2)]
            columns.pack1(left, True, False)
            columns.pack2(right, True, False)
            left.pack_start(Gtk.Label(label='Task sequence — double-click a task to edit', xalign=0), False, False, 0)
            self.tasks = Gtk.ListStore(str)
            self.task_view = Gtk.TreeView(model=self.tasks)
            self.task_view.append_column(Gtk.TreeViewColumn('Tasks', Gtk.CellRendererText(), text=0))
            self.task_view.connect('row-activated', self.edit_task)
            self.scrolled(left, self.task_view)
            controls = Gtk.Box(spacing=8)
            self.add_button = self.button('+', self.add_task, controls)
            self.add_button.set_tooltip_text('Append a task using the previous task angles (first task uses the live robot state).')
            self.remove_button = self.button('−', self.remove_task, controls)
            self.remove_button.set_tooltip_text('Delete the selected task.')
            left.pack_start(controls, False, False, 0)
            right.pack_start(Gtk.Label(label='Recorded robot states', xalign=0), False, False, 0)
            captures = Gtk.Box(spacing=8)
            self.capture_button = self.button('Capture RViz state', lambda _: self.capture('rviz'), captures)
            self.current_button = self.button('Capture live state', lambda _: self.capture('current'), captures)
            right.pack_start(captures, False, False, 0)
            self.recordings = Gtk.ListStore(str)
            self.state_view = Gtk.TreeView(model=self.recordings)
            self.state_view.append_column(Gtk.TreeViewColumn('States', Gtk.CellRendererText(), text=0))
            self.state_view.get_selection().connect('changed', self.show_state)
            self.scrolled(right, self.state_view)
            self.details = Gtk.Label(xalign=0, selectable=True)
            self.details.set_line_wrap(True)
            self.details.set_text('Set the goal pose in RViz, then capture it here.\nSelect a recording in the task window to read its hand or arm goal.')
            self.scrolled(right, self.details)
            footer = Gtk.Box(spacing=10)
            self.plan_button = self.button('Plan, execute & save', lambda _: self.plan(True), footer)
            self.plan_button.get_style_context().add_class('suggested-action')
            self.save_button = self.button('Plan & save', lambda _: self.plan(False), footer)
            outer.pack_start(footer, False, False, 0)
            self.status = Gtk.Label(label='Reading initial robot joint angles…', xalign=0)
            self.status.set_line_wrap(True)
            outer.pack_start(self.status, False, False, 0)
            self.show_all()
            self.capture('current', initial=True)

        def button(self, label, callback, box):
            button = Gtk.Button(label=label)
            button.connect('clicked', callback)
            box.pack_start(button, False, False, 0)
            return button

        def scrolled(self, box, widget):
            scroll = Gtk.ScrolledWindow()
            scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
            scroll.add(widget)
            box.pack_start(scroll, True, True, 0)

        def refresh(self):
            ready = panel.session.ready and not (panel.busy or panel.stopping or panel.resetting)
            for button in (self.capture_button, self.current_button):
                button.set_sensitive(ready)
            for button in (self.add_button, self.remove_button, self.plan_button, self.save_button):
                button.set_sensitive(ready and self.sequence is not None)
            for button in (self.plan_button, self.save_button):
                button.set_sensitive(ready and self.sequence is not None and bool(self.sequence.tasks))
            self.task_view.set_sensitive(ready and self.sequence is not None)
            self.name.set_sensitive(ready)

        def capture(self, source, initial=False):
            self.status.set_text('Reading robot state…')
            def received(state):
                if self.closed:
                    return
                self.bounds = state['bounds']
                if self.sequence is None:
                    self.sequence = Sequence(state)
                if not initial:
                    recording = self.sequence.capture(state)
                    self.recordings.append([recording['name'] + ' — ' + source])
                    self.state_view.set_cursor(len(self.sequence.recordings) - 1)
                self.status.set_text('State captured.' if not initial else 'Add tasks with +. The first task starts with these live angles.')
                self.refresh()
            panel.work(lambda: panel.session.capture_state(source), received)

        def add_task(self, _):
            if self.sequence is None:
                return
            self.sequence.add()
            self.refresh_tasks(len(self.sequence.tasks) - 1)

        def selected(self, view):
            model, iterator = view.get_selection().get_selected()
            return model.get_path(iterator).get_indices()[0] if iterator is not None else None

        def remove_task(self, _):
            index = self.selected(self.task_view)
            if index is not None:
                del self.sequence.tasks[index]
                self.refresh_tasks(min(index, len(self.sequence.tasks) - 1))

        def refresh_tasks(self, selected=0):
            self.tasks.clear()
            for number, task in enumerate(self.sequence.tasks, 1):
                self.tasks.append(['%d. %s (%s)' % (number, task['name'], task['order'].replace('_', ' → '))])
            if self.sequence.tasks:
                self.task_view.set_cursor(max(0, selected))
            self.refresh()

        def show_state(self, _):
            index = self.selected(self.state_view)
            if index is None or self.sequence is None:
                return
            state = self.sequence.recordings[index]
            lines = [state['name'] + ' — angles in degrees']
            for group in ('arm', 'hand'):
                lines.append(group.capitalize())
                lines.extend('%s: %.3f°' % (name, math.degrees(value)) for name, value in state[group].items())
            self.details.set_text('\n'.join(lines))

        def edit_task(self, _, path, column):
            if self.sequence is None or panel.busy or not panel.session.ready:
                return
            index = path.get_indices()[0]
            original = self.sequence.tasks[index]
            dialog = Gtk.Dialog(title='Edit ' + original['name'], transient_for=self, modal=True)
            dialog.set_default_size(850, 650)
            dialog.add_buttons('Cancel', Gtk.ResponseType.CANCEL, 'Apply', Gtk.ResponseType.OK)
            body = dialog.get_content_area()
            body.set_spacing(10)
            body.set_border_width(12)
            title = Gtk.Entry(text=original['name'])
            body.pack_start(title, False, False, 0)
            order = Gtk.ComboBoxText()
            order.append('arm_hand', 'Move arm, then hand')
            order.append('hand_arm', 'Move hand, then arm')
            order.set_active_id(original['order'])
            body.pack_start(order, False, False, 0)
            states = Gtk.ComboBoxText()
            for number, state in enumerate(self.sequence.recordings):
                states.append(str(number), state['name'] + ' — ' + state['source'])
            selected = self.selected(self.state_view)
            if self.sequence.recordings:
                states.set_active(selected if selected is not None else len(self.sequence.recordings) - 1)
            body.pack_start(Gtk.Label(label='Recorded state to read:', xalign=0), False, False, 0)
            body.pack_start(states, False, False, 0)
            columns = Gtk.Box(spacing=20)
            body.pack_start(columns, True, True, 0)
            fields = {'arm': {}, 'hand': {}}
            for group in ('arm', 'hand'):
                box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
                columns.pack_start(box, True, True, 0)
                button = self.button('Read %s goal' % group, lambda _, g=group: read(g), box)
                button.set_sensitive(bool(self.sequence.recordings))
                grid = Gtk.Grid(column_spacing=12, row_spacing=6)
                for row, (name, value) in enumerate(original[group].items()):
                    field = Gtk.Entry(text='%.10f' % math.degrees(value))
                    field.set_width_chars(14)
                    lo, hi = self.bounds[name]
                    field.set_tooltip_text('Limits: %.3f° to %.3f°' % (math.degrees(lo), math.degrees(hi)))
                    grid.attach(Gtk.Label(label=name, xalign=0), 0, row, 1, 1)
                    grid.attach(field, 1, row, 1, 1)
                    fields[group][name] = field
                self.scrolled(box, grid)
            error = Gtk.Label(label='All joint angles are in degrees.', xalign=0)
            error.set_line_wrap(True)
            body.pack_start(error, False, False, 0)
            def read(group):
                selection = states.get_active_id()
                if selection is None:
                    error.set_text('Capture and select a recorded state first.')
                    return
                recording = self.sequence.recordings[int(selection)]
                for name, field in fields[group].items():
                    field.set_text('%.10f' % math.degrees(recording[group][name]))
                error.set_text('%s goal read from %s.' % (group.capitalize(), recording['name']))
            dialog.show_all()
            while dialog.run() == Gtk.ResponseType.OK:
                if panel.busy or not panel.session.ready:
                    error.set_text('Wait until simulation is ready before applying this task.')
                    continue
                try:
                    task = copy.deepcopy(original)
                    task.update(name=title.get_text().strip(), order=order.get_active_id())
                    if not task['name']:
                        raise ValueError('Enter a task name.')
                    for group, entries in fields.items():
                        for name, field in entries.items():
                            angle = math.radians(float(field.get_text()))
                            lo, hi = self.bounds[name]
                            if not math.isfinite(angle) or angle < lo - 1e-9 or angle > hi + 1e-9:
                                raise ValueError('%s must be between %.3f° and %.3f°.' % (name, math.degrees(lo), math.degrees(hi)))
                            task[group][name] = max(lo, min(hi, angle))
                    self.sequence.tasks[index] = task
                    self.refresh_tasks(index)
                    break
                except ValueError as exc:
                    error.set_text(str(exc))
            dialog.destroy()

        def plan(self, execute):
            if self.sequence is None:
                return
            try:
                document = self.sequence.document(self.name.get_text())
            except ValueError as exc:
                self.status.set_text(str(exc))
                return
            document['recordings'] = copy.deepcopy(self.sequence.recordings)
            def run():
                runtime = ROOT / '.runtime'
                runtime.mkdir(exist_ok=True)
                with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix='custom-', dir=runtime, delete=False) as stream:
                    json.dump(document, stream, indent=2)
                    filename = Path(stream.name)
                try:
                    args = ['--sequence', '/project/' + filename.relative_to(ROOT).as_posix()]
                    if execute:
                        args.append('--execute')
                    panel.session.terminal_task('custom-plan', *args)
                finally:
                    filename.unlink(missing_ok=True)
            self.status.set_text('Planning custom trajectory… Follow progress in the task terminal.')
            panel.work(run, lambda _: self.completed())

        def completed(self):
            if not self.closed:
                self.status.set_text('Task terminal closed. Saved runs are listed in the main panel; check the terminal result for success.')
                panel.refresh_runs()
                self.refresh()

    return Editor()
