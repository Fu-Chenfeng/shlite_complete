"""Native Linux desktop controls for simulation and real hardware."""
import fcntl
import subprocess
import threading

from desktop_session import Cancelled, ROOT, Session
from trajectory_store import replay_run_choices


def main(mode='sim'):
    try:
        import gi
        gi.require_version('Gtk', '3.0')
        from gi.repository import GLib, Gtk
    except (ImportError, ValueError) as exc:
        raise RuntimeError('Desktop controls need python3-gi and gir1.2-gtk-3.0.') from exc
    if mode not in ('sim', 'real'):
        raise ValueError('Use panel sim or panel real')
    runtime = ROOT / '.runtime'
    runtime.mkdir(exist_ok=True)
    lockfile = (runtime / ('panel-' + mode + '.lock')).open('w')
    try:
        fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        dialog = Gtk.MessageDialog(text='This control panel is already open.',
                                   buttons=Gtk.ButtonsType.OK)
        dialog.run()
        dialog.destroy()
        return

    class Panel(Gtk.Window):
        def __init__(self):
            super().__init__(title='SH Lite — ' + ('Simulation' if mode == 'sim' else 'Real Control'))
            self.set_default_size(850, 600)
            self.set_border_width(18)
            self.session = Session(mode, self.log)
            self.busy = False
            self.stopping = False
            self.resetting = False
            self.operation_generation = 0
            self.failed_processes = set()
            self.buttons = []
            outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
            self.add(outer)
            title = Gtk.Label(xalign=0)
            title.set_markup('<big><b>UR5 + Shadow Hand Lite</b></big>')
            outer.pack_start(title, False, False, 0)
            note = ('Five stages: approach → close → tighten → lift → home and open.' if mode == 'sim'
                    else 'Start hardware with the hand clear for calibration. Start External Control on the UR5 pendant when prompted.')
            label = Gtk.Label(label=note, xalign=0)
            label.set_line_wrap(True)
            outer.pack_start(label, False, False, 0)
            grid = Gtk.Grid(column_spacing=10, row_spacing=10)
            outer.pack_start(grid, False, False, 0)
            self.start_button = self.button('Start simulation' if mode == 'sim' else 'Start real hardware',
                                            self.session.start, grid, 0, 0, needs_ready=False)
            self.start_button.get_style_context().add_class('suggested-action')
            if mode == 'sim':
                self.button('Plan, execute & save', lambda: self.session.terminal_task('plan'), grid, 1, 0,
                            tooltip='Plans all five stages and saves a new run named after the GraspIt file (or timestamp for manual mode).')
                self.reset_button = Gtk.Button(label='Reset simulation')
                self.reset_button.connect('clicked', self.reset)
                self.reset_button.set_tooltip_text('Cancel the current task and restart simulation at its configured starting state.')
                grid.attach(self.reset_button, 0, 2, 1, 1)
            self.check_button = self.button('Check saved task', lambda: self.saved_task('check'), grid, 0, 1, background=False)
            self.replay_button = self.button('Replay five-stage task…', lambda: self.saved_task('replay'), grid, 1, 1,
                        tooltip='Opens a terminal. Press Enter before each stage; q quits between stages.', background=False)
            self.button('Status', lambda: self.session.run(mode, 'status'), grid, 2, 1, needs_ready=False)
            edit = Gtk.Button(label='Edit config.yaml')
            edit.connect('clicked', lambda _: subprocess.Popen(['xdg-open', str(ROOT / 'config.yaml')]))
            grid.attach(edit, 2, 0, 1, 1)
            saved = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
            saved.pack_start(Gtk.Label(label='Saved trajectories:'), False, False, 0)
            self.replay_choice = Gtk.ComboBoxText()
            self.replay_choice.set_hexpand(True)
            self.replay_choice.connect('changed', lambda _: self.refresh())
            self.run_choices = None
            self.available_runs = set()
            saved.pack_start(self.replay_choice, True, True, 0)
            refresh = Gtk.Button(label='Refresh list')
            refresh.connect('clicked', lambda _: self.refresh_runs())
            saved.pack_start(refresh, False, False, 0)
            outer.pack_start(saved, False, False, 0)
            self.stop_button = Gtk.Button(label='Shut down simulation' if mode == 'sim' else 'Shut down hardware')
            self.stop_button.connect('clicked', self.stop)
            outer.pack_start(self.stop_button, False, False, 0)
            self.status = Gtk.Label(label='Ready to start. Edit config.yaml to change settings.', xalign=0)
            self.status.set_line_wrap(True)
            outer.pack_start(self.status, False, False, 0)
            scroll = Gtk.ScrolledWindow()
            scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
            self.view = Gtk.TextView(editable=False, monospace=True, wrap_mode=Gtk.WrapMode.WORD_CHAR)
            self.buffer = self.view.get_buffer()
            scroll.add(self.view)
            outer.pack_start(scroll, True, True, 0)
            footer = Gtk.Label(label='Scene and robot edits apply after restart. Closing this panel requires session shutdown.', xalign=0)
            footer.set_line_wrap(True)
            outer.pack_start(footer, False, False, 0)
            self.connect('delete-event', self.close)
            self.connect('destroy', Gtk.main_quit)
            GLib.timeout_add_seconds(2, self.monitor)
            self.refresh_runs()
            self.refresh()

        def button(self, label, action, grid, col, row, needs_ready=True, tooltip=None, background=True):
            button = Gtk.Button(label=label)
            if tooltip:
                button.set_tooltip_text(tooltip)
            button.connect('clicked', lambda _: self.work(action) if background else action())
            grid.attach(button, col, row, 1, 1)
            self.buttons.append((button, needs_ready))
            return button

        def refresh(self):
            idle = not self.busy and not self.stopping and not self.resetting
            for button, needs_ready in self.buttons:
                button.set_sensitive(idle and (self.session.ready or not needs_ready))
            self.start_button.set_sensitive(idle and not self.session.active)
            if mode == 'sim':
                self.reset_button.set_sensitive(not self.stopping and not self.resetting)
            if hasattr(self, 'replay_choice'):
                selection = self.replay_choice.get_active_id()
                playable = bool(self.available_runs) if selection == 'latest' else selection in self.available_runs
                for button in (self.check_button, self.replay_button):
                    button.set_sensitive(idle and self.session.ready and playable)
                self.replay_choice.set_sensitive(idle)
            self.stop_button.set_sensitive(not self.stopping if mode == 'sim' else
                                           self.session.active and not self.stopping and not self.session.task_active)
            self.stop_button.set_tooltip_text('Stops simulation even during startup or an open task terminal.'
                                             if mode == 'sim' else
                                             'Finish or quit the task and close its terminal before shutdown.'
                                             if self.session.task_active else None)

        def refresh_runs(self):
            choices = replay_run_choices(ROOT / 'trajectories')
            if choices == self.run_choices:
                return
            selection = self.replay_choice.get_active_id()
            if selection is None:
                # Read only this preference; malformed unrelated config must not
                # prevent the panel's shutdown/reset controls from opening.
                try:
                    import yaml
                    selection = yaml.safe_load((ROOT / 'config.yaml').read_text()).get('trajectories', {}).get('replay_run', 'latest')
                except (OSError, ValueError, AttributeError, yaml.YAMLError):
                    selection = 'latest'
            self.run_choices = choices
            self.available_runs = {entry['name'] for entry in choices if entry['replayable']}
            self.replay_choice.remove_all()
            self.replay_choice.append('latest', 'Latest completed run' if self.available_runs else 'No completed compatible runs yet')
            for entry in choices:
                self.replay_choice.append(entry['name'], '%s — %s' % (entry['name'], entry['status']))
            if not isinstance(selection, str) or not self.replay_choice.set_active_id(selection):
                self.replay_choice.set_active_id('latest')
            self.refresh()

        def saved_task(self, command):
            selection = self.replay_choice.get_active_id() or 'latest'
            self.work(lambda: self.session.replay(command, selection))

        def log(self, message):
            GLib.idle_add(self.append_log, message)

        def append_log(self, message):
            self.buffer.insert(self.buffer.get_end_iter(), message + '\n')
            lines = self.buffer.get_line_count()
            if lines > 1800:
                self.buffer.delete(self.buffer.get_start_iter(), self.buffer.get_iter_at_line(lines - 1800))
            self.view.scroll_to_iter(self.buffer.get_end_iter(), 0, False, 0, 1)
            return False

        def work(self, action):
            if self.busy or self.stopping or self.resetting:
                return
            self.busy = True
            generation = self.operation_generation
            self.status.set_text('Working… See progress below or in the task terminal.')
            self.refresh()

            def run():
                message = None
                try:
                    with self.session.lock:
                        if generation != self.operation_generation:
                            raise Cancelled('Queued task cancelled by reset or shutdown.')
                        self.session.check_cancelled()
                        action()
                except Cancelled as exc:
                    self.log(str(exc))
                except Exception as exc:
                    message = str(exc)
                    self.log('ERROR: ' + message)
                GLib.idle_add(self.finished, message, generation)

            threading.Thread(target=run, daemon=True).start()

        def finished(self, error, generation):
            if generation != self.operation_generation:
                return False
            self.busy = False
            if error and not self.stopping and not self.resetting:
                self.status.set_text(error)
            elif not self.stopping and not self.resetting:
                self.status.set_text('Session ready.' if self.session.ready else 'Session stopped / ready to start.')
            self.refresh()
            return False

        def reset(self, _):
            if self.resetting or self.stopping:
                return
            self.resetting = True
            self.operation_generation += 1
            generation = self.operation_generation
            self.session.request_reset()
            self.status.set_text('Resetting simulation… Cancelling the current task and restoring the starting scene.')
            self.refresh()

            def restart():
                error = None
                try:
                    with self.session.lock:
                        if generation != self.operation_generation:
                            raise Cancelled('Reset cancelled by shutdown.')
                        self.session.reset()
                except Exception as exc:
                    error = str(exc)
                    self.log('RESET: ' + error)
                GLib.idle_add(self.reset_finished, error, generation)

            threading.Thread(target=restart, daemon=True).start()

        def reset_finished(self, error, generation):
            if generation != self.operation_generation:
                return False
            self.resetting = False
            self.busy = False
            if not self.stopping:
                self.status.set_text(error or 'Simulation reset and ready.')
            self.refresh_runs()
            self.refresh()
            return False

        def stop(self, _):
            if self.stopping or (mode == 'real' and self.session.task_active):
                return
            self.stopping = True
            self.operation_generation += 1
            self.session.cancel.set()
            self.status.set_text('Shutting down… Keep this panel open until shutdown finishes.')
            self.refresh()

            def shutdown():
                error = None
                try:
                    with self.session.lock:
                        self.session.shutdown()
                except Exception as exc:
                    error = str(exc)
                    self.log('SHUTDOWN ERROR: ' + error)
                GLib.idle_add(self.stopped, error)

            threading.Thread(target=shutdown, daemon=True).start()

        def stopped(self, error):
            self.stopping = False
            self.resetting = False
            self.busy = False
            self.status.set_text(error or 'Shutdown complete. You can close the panel.')
            self.refresh()
            return False

        def monitor(self):
            if not self.busy and not self.resetting and not self.stopping:
                self.refresh_runs()
            self.refresh()
            if not self.busy and not self.stopping and not self.resetting:
                for label, process in self.session.children:
                    if process.poll() is not None and process.pid not in self.failed_processes:
                        self.failed_processes.add(process.pid)
                        self.session.ready = False
                        self.log('%s exited. Shut down this session before restarting.' % label)
                        self.status.set_text('A session process exited. Inspect the log, then shut down.')
            return True

        def close(self, *_):
            if self.session.active or self.busy or self.stopping or self.resetting:
                self.status.set_text('Use Shut down first, then close this panel. You can minimize it while working.')
                return True
            return False

    panel = Panel()
    panel.show_all()
    Gtk.main()
    lockfile.close()


if __name__ == '__main__':
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else 'sim')
