"""Generate self-locating launchers inside the project. No installation needed."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def script_entries():
    for filename, mode in (('Simulation.sh', 'sim'), ('Real-Control.sh', 'real')):
        yield filename, ('#!/bin/sh\n'
                         '# Right-click this file in Ubuntu Files and choose Run as a Program.\n'
                         'project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd) || exit 1\n'
                         'exec /usr/bin/python3 "$project_dir/robotctl" panel ' + mode + '\n')


def create_launchers():
    for filename, contents in script_entries():
        target = ROOT / filename
        target.write_text(contents)
        target.chmod(0o755)
        print('Created project launcher:', target)


if __name__ == '__main__':
    create_launchers()
