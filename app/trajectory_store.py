"""Named arm/hand recording sessions, shared by planning and replay."""
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import re

TOKYO = timezone(timedelta(hours=9), 'Asia/Tokyo')
RUN_NAME = re.compile(r'^[\w][\w .()-]*$')
DEFAULT_ROOT = Path(__file__).resolve().parents[1] / 'trajectories'


def archive_root():
    return Path(os.environ.get('TRAJECTORY_ROOT', str(DEFAULT_ROOT))).expanduser()


def write_metadata(run, metadata):
    temporary = run / 'run.json.tmp'
    temporary.write_text(json.dumps(metadata, indent=2) + '\n')
    temporary.replace(run / 'run.json')


def create_run(root=None, now=None, source='planned', task_version=None, grasp_file=None):
    root = Path(root) if root is not None else archive_root()
    root.mkdir(parents=True, exist_ok=True)
    now = now or datetime.now(TOKYO)
    if now.tzinfo is None:
        raise ValueError('Recording timestamps must include a timezone')
    now = now.astimezone(TOKYO)
    name = Path(grasp_file).stem if grasp_file else now.strftime('%Y-%m-%d_%H-%M-%S')
    if not RUN_NAME.fullmatch(name):
        raise ValueError('Grasp filename cannot be used as a trajectory folder name: %s' % name)
    suffix = 1 if name == 'latest' else 0  # "latest" is the replay selection keyword.
    while True:
        run = root / (name if not suffix else '%s_%03d' % (name, suffix))
        try:
            run.mkdir()
            break
        except FileExistsError:
            suffix += 1
    for backend in ('arm', 'hand'):
        (run / backend).mkdir()
    metadata = {'created_at': now.isoformat(), 'timezone': 'Asia/Tokyo',
                'status': 'recording', 'source': source}
    if task_version is not None:
        metadata['task_version'] = task_version
    if grasp_file:
        metadata['grasp_file'] = str(grasp_file)
    write_metadata(run, metadata)
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


def matches_task(run, task_version):
    if task_version is None:
        return True
    try:
        return json.loads((run / 'run.json').read_text()).get('task_version') == task_version
    except (OSError, ValueError, AttributeError):
        return False


def recording_order(run):
    """Newest by creation time, independent of the grasp's alphabetical name."""
    try:
        stamp = datetime.fromisoformat(json.loads((run / 'run.json').read_text())['created_at'])
        if stamp.tzinfo is None:
            raise ValueError('Missing timestamp timezone')
        return stamp.timestamp(), run.name
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return run.stat().st_mtime, run.name


def resolve_run(root=None, selection='latest', task_version=None):
    """Resolve once before replay, so all stages come from the same recording."""
    root = Path(root) if root is not None else archive_root()
    if selection != 'latest':
        if not isinstance(selection, str) or not RUN_NAME.fullmatch(selection):
            raise ValueError('trajectories.replay_run must be latest or a saved folder name')
        run = root / selection
        if not is_complete(run):
            raise ValueError('Selected trajectory run is missing or incomplete: %s' % run)
        if not matches_task(run, task_version):
            raise ValueError('Selected recording uses the old task. Generate and select a new five-stage run.')
        return run
    # Also accept an explicitly supplied run directory (and the old direct layout).
    if all((root / backend).is_dir() for backend in ('arm', 'hand')):
        if (root / 'run.json').exists() and not is_complete(root):
            raise ValueError('Trajectory run is incomplete: %s' % root)
        if not matches_task(root, task_version):
            raise ValueError('This recording uses the old task. Generate a new five-stage run.')
        return root
    if root.is_dir():
        for candidate in sorted(root.iterdir(), key=recording_order, reverse=True):
            if RUN_NAME.fullmatch(candidate.name) and is_complete(candidate) and matches_task(candidate, task_version):
                return candidate
    raise ValueError('No completed compatible trajectory runs in %s. Generate a new plan first.' % root)


def configured_run(task_version=None):
    from project_config import load_config
    selection = load_config().get('trajectories', {}).get('replay_run', 'latest')
    return resolve_run(selection=selection, task_version=task_version)


def replay_run_choices(root=None):
    """Describe saved runs for the panel without deserializing trajectory pickles."""
    from task_definition import TASK_VERSION, actions, load_task
    root = Path(root) if root is not None else archive_root()
    if not root.is_dir():
        return []
    choices = []
    for run in sorted(root.iterdir(), key=recording_order, reverse=True):
        if not run.is_dir() or not RUN_NAME.fullmatch(run.name):
            continue
        try:
            metadata = json.loads((run / 'run.json').read_text())
            status = metadata.get('status', 'incomplete')
            if status != 'complete':
                reason = status
            elif metadata.get('task_version') != TASK_VERSION:
                reason = 'older sequence'
            else:
                load_task(run)
                required = [(action['group'], action['file']) for action in actions()]
                required += [('hand', action['target']) for action in actions() if 'target' in action]
                reason = 'ready' if all((run / group / filename).is_file() for group, filename in required) else 'missing files'
        except (OSError, ValueError, TypeError, AttributeError):
            reason = 'invalid recording'
        choices.append({'name': run.name, 'status': reason, 'replayable': reason == 'ready'})
    return choices
