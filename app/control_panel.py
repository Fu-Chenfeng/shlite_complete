"""Native Linux desktop controls for simulation and real hardware."""
import fcntl
import subprocess
import threading

from desktop_session import Cancelled, ROOT, Session


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
            self.failed_processes = set()
            self.buttons = []
            outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
            self.add(outer)
            title = Gtk.Label(xalign=0)
            title.set_markup('<big><b>UR5 + Shadow Hand Lite</b></big>')
            outer.pack_start(title, False, False, 0)
            note = ('Start the simulation, then plan a new task or replay saved stages.' if mode == 'sim'
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
                            tooltip='Plans and executes in simulation, saving a new timestamped trajectory folder.')
            self.button('Check saved task', lambda: self.session.run(mode, 'check'), grid, 0, 1)
            self.button('Replay saved task…', lambda: self.session.terminal_task('replay'), grid, 1, 1,
                        tooltip='Opens a terminal. Press Enter before each stage; q quits between stages.')
            self.button('Status', lambda: self.session.run(mode, 'status'), grid, 2, 1, needs_ready=False)
            edit = Gtk.Button(label='Edit config.yaml')
            edit.connect('clicked', lambda _: subprocess.Popen(['xdg-open', str(ROOT / 'config.yaml')]))
            grid.attach(edit, 2, 0, 1, 1)
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
            self.refresh()

        def button(self, label, action, grid, col, row, needs_ready=True, tooltip=None):
            button = Gtk.Button(label=label)
            if tooltip:
                button.set_tooltip_text(tooltip)
            button.connect('clicked', lambda _: self.work(action))
            grid.attach(button, col, row, 1, 1)
            self.buttons.append((button, needs_ready))
            return button

        def refresh(self):
            for button, needs_ready in self.buttons:
                button.set_sensitive(not self.busy and (self.session.ready or not needs_ready))
            self.start_button.set_sensitive(not self.busy and not self.session.active)
            self.stop_button.set_sensitive(self.session.active and not self.stopping and not self.session.task_active)
            self.stop_button.set_tooltip_text('Finish or quit the task and close its terminal before shutdown.'
                                             if self.session.task_active else None)

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
            if self.busy:
                return
            self.busy = True
            self.status.set_text('Working… See progress below or in the task terminal.')
            self.refresh()

            def run():
                message = None
                try:
                    with self.session.lock:
                        action()
                except Cancelled as exc:
                    self.log(str(exc))
                except Exception as exc:
                    message = str(exc)
                    self.log('ERROR: ' + message)
                GLib.idle_add(self.finished, message)

            threading.Thread(target=run, daemon=True).start()

        def finished(self, error):
            self.busy = False
            if error:
                self.status.set_text(error)
            elif not self.stopping:
                self.status.set_text('Session ready.' if self.session.ready else 'Session stopped / ready to start.')
            self.refresh()
            return False

        def stop(self, _):
            if self.stopping or self.session.task_active:
                return
            self.stopping = True
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
            self.busy = False
            self.status.set_text(error or 'Shutdown complete. You can close the panel.')
            self.refresh()
            return False

        def monitor(self):
            self.refresh()
            if not self.busy and not self.stopping:
                for label, process in self.session.children:
                    if process.poll() is not None and process.pid not in self.failed_processes:
                        self.failed_processes.add(process.pid)
                        self.session.ready = False
                        self.log('%s exited. Shut down this session before restarting.' % label)
                        self.status.set_text('A session process exited. Inspect the log, then shut down.')
            return True

        def close(self, *_):
            if self.session.active or self.busy or self.stopping:
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
