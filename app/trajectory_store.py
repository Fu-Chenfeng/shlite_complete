"""Timestamped arm/hand recording sessions, shared by planning and replay."""
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import re

TOKYO = timezone(timedelta(hours=9), 'Asia/Tokyo')
RUN_NAME = re.compile(r'^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}(?:_\d+)?$')
DEFAULT_ROOT = Path(__file__).resolve().parents[1] / 'trajectories'


def archive_root():
    return Path(os.environ.get('TRAJECTORY_ROOT', str(DEFAULT_ROOT))).expanduser()


def write_metadata(run, metadata):
    temporary = run / 'run.json.tmp'
    temporary.write_text(json.dumps(metadata, indent=2) + '\n')
    temporary.replace(run / 'run.json')


def create_run(root=None, now=None, source='planned'):
    root = Path(root) if root is not None else archive_root()
    root.mkdir(parents=True, exist_ok=True)
    now = now or datetime.now(TOKYO)
    if now.tzinfo is None:
        raise ValueError('Recording timestamps must include a timezone')
    now = now.astimezone(TOKYO)
    name = now.strftime('%Y-%m-%d_%H-%M-%S')
    suffix = 0
    while True:
        run = root / (name if not suffix else '%s_%03d' % (name, suffix))
        try:
            run.mkdir()
            break
        except FileExistsError:
            suffix += 1
    for backend in ('arm', 'hand'):
        (run / backend).mkdir()
    write_metadata(run, {'created_at': now.isoformat(), 'timezone': 'Asia/Tokyo',
                         'status': 'recording', 'source': source})
    return run


def finish_run(run, successful):
    run = Path(run)
    metadata = json.loads((run / 'run.json').read_text())
    metadata.update(status='complete' if successful else 'failed',
                    finished_at=datetime.now(TOKYO).isoformat())
    write_metadata(run, metadata)


def is_complete(run):
    try:
        return (all((run / backend).is_dir() for backend in ('arm', 'hand')) and
                json.loads((run / 'run.json').read_text())['status'] == 'complete')
    except (OSError, ValueError, KeyError, TypeError):
        return False


def resolve_run(root=None, selection='latest'):
    """Resolve once before replay, so all stages come from the same recording."""
    root = Path(root) if root is not None else archive_root()
    if selection != 'latest':
        if not isinstance(selection, str) or not RUN_NAME.fullmatch(selection):
            raise ValueError('trajectories.replay_run must be latest or a timestamp folder name')
        run = root / selection
        if not is_complete(run):
            raise ValueError('Selected trajectory run is missing or incomplete: %s' % run)
        return run
    # Also accept an explicitly supplied run directory (and the old direct layout).
    if all((root / backend).is_dir() for backend in ('arm', 'hand')):
        if (root / 'run.json').exists() and not is_complete(root):
            raise ValueError('Trajectory run is incomplete: %s' % root)
        return root
    if root.is_dir():
        for candidate in sorted(root.iterdir(), reverse=True):
            if RUN_NAME.fullmatch(candidate.name) and is_complete(candidate):
                return candidate
    raise ValueError('No completed trajectory runs in %s. Generate a new plan first.' % root)


def configured_run():
    from project_config import load_config
    selection = load_config().get('trajectories', {}).get('replay_run', 'latest')
    return resolve_run(selection=selection)
