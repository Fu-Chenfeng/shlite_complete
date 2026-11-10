"""Desktop session orchestration; leaves robot commands in robotctl."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


class Cancelled(RuntimeError):
    pass


class Session:
    def __init__(self, mode, output=print):
        if mode not in ('sim', 'real'):
            raise ValueError('Choose sim or real')
        self.mode = mode
        self.output = output
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.children = []
        self.active = False
        self.ready = False
        self.task_active = False

    def command(self, *args):
        return [sys.executable, str(ROOT / 'robotctl'), *args]

    def spawn(self, args, label):
        process = subprocess.Popen(args, cwd=ROOT, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1,
                                   start_new_session=True)

        def read():
            with process.stdout:
                for line in process.stdout:
                    self.output('[%s] %s' % (label, line.rstrip()))

        threading.Thread(target=read, daemon=True).start()
        return process

    def ensure_running(self):
        for label, process in self.children:
            if process.poll() is not None:
                self.ready = False
                raise RuntimeError('%s exited (code %d). Inspect the log, then shut down before restarting.' %
                                   (label, process.returncode))

    def run(self, *args, timeout=240, stopping=False):
        if not stopping:
            self.check_cancelled()
            self.ensure_running()
        process = self.spawn(self.command(*args), ' '.join(args))
        deadline = time.monotonic() + timeout
        try:
            while process.poll() is None:
                if not stopping:
                    self.check_cancelled()
                    self.ensure_running()
                if time.monotonic() >= deadline:
                    raise RuntimeError('Timed out: ' + ' '.join(args))
                time.sleep(0.1)
            if process.returncode:
                raise RuntimeError('%s failed (exit %d). See the log below.' % (' '.join(args), process.returncode))
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

    def check_cancelled(self):
        if self.cancel.is_set():
            raise Cancelled('Startup/task cancelled; shutting down the session.')

    def launch(self, command):
        self.check_cancelled()
        self.ensure_running()
        self.output('Starting ' + command + '…')
        process = self.spawn(self.command(self.mode, command), command)
        self.children.append((command, process))

    def start(self):
        if self.active:
            raise RuntimeError('A session has already been started. Shut it down before restarting.')
        self.cancel.clear()
        self.run('init')
        self.run('validate-config')
        self.run('gui')
        # Also blocks startup against a ROS session outside this project.
        self.run(self.mode, 'preflight')
        self.active = True
        self.run(self.mode, 'up')
        if self.mode == 'sim':
            self.launch('start')
            self.run('sim', 'wait', 'sim')
        else:
            self.launch('core')
            self.run('real', 'wait-core')
            self.output('Starting hand hardware. Keep the hand clear during calibration.')
            self.launch('nuc')
            self.run('real', 'wait', 'hand')
            self.output('Start the External Control program on the UR5 pendant when its driver is ready.')
            self.launch('ur5')
            self.run('real', 'wait', 'ur5')
            self.launch('adapter')
            self.launch('moveit')
            self.output('Start the External Control program on the UR5 pendant. Waiting for robot readiness…')
            self.run('real', 'wait', 'real')
            self.run('real', 'obstacles')
        self.run(self.mode, 'collisions')
        self.ready = True
        self.output('Session ready. Choose a task below.')

    def shutdown(self):
        # Never terminate ROS core before the normal hand shutdown calibration.
        command = 'stop-idle' if self.mode == 'real' and not any(label == 'nuc' for label, _ in self.children) else 'shutdown'
        self.run(self.mode, command, timeout=240, stopping=True)
        for _, process in self.children:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.output('A session process is still exiting; keep this panel open.')
                raise RuntimeError('Shutdown did not finish. Inspect the log and retry.')
        self.children.clear()
        self.active = self.ready = False
        self.cancel.clear()
        self.output('Session shut down. You can close this panel.')

    def terminal_task(self, command):
        self.check_cancelled()
        self.ensure_running()
        if not self.ready:
            raise RuntimeError('Start the session and wait for readiness first.')
        argv = [sys.executable, str(ROOT / 'app/terminal_task.py'), self.mode, command]
        terminal = shutil.which('gnome-terminal') or shutil.which('x-terminal-emulator')
        if not terminal:
            raise RuntimeError('Install a desktop terminal to use interactive tasks.')
        prefix = ['--wait', '--'] if Path(terminal).name == 'gnome-terminal' else ['-e']
        self.output('Task opened in a terminal. Follow its prompts; close it when finished.')
        self.task_active = True
        try:
            result = subprocess.run([terminal, *prefix, *argv], cwd=ROOT)
            if result.returncode:
                raise RuntimeError('The terminal/task exited with code %d. Check its output.' % result.returncode)
        finally:
            self.task_active = False
