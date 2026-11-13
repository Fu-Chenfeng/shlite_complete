"""Host-only checks for recording isolation, timestamps and replay selection."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
from trajectory_store import create_run, finish_run, resolve_run, replay_run_choices


class TrajectoryStoreTest(unittest.TestCase):
    def test_dropdown_lists_ready_failed_legacy_and_missing_recordings(self):
        from task_definition import TASK_VERSION, STAGES, actions
        ready = create_run(self.root, task_version=TASK_VERSION)
        (ready / 'task.json').write_text(json.dumps({'version': TASK_VERSION, 'stages': STAGES}))
        for action in actions():
            (ready / action['group'] / action['file']).write_bytes(b'not deserialized by listing')
            if 'target' in action:
                (ready / 'hand' / action['target']).write_bytes(b'not deserialized by listing')
        finish_run(ready, True)
        failed = create_run(self.root, task_version=TASK_VERSION)
        finish_run(failed, False)
        legacy = create_run(self.root)
        finish_run(legacy, True)
        missing = create_run(self.root, task_version=TASK_VERSION)
        (missing / 'task.json').write_text((ready / 'task.json').read_text())
        finish_run(missing, True)
        broken = create_run(self.root)
        (broken / 'run.json').write_text('[]')
        listed = replay_run_choices(self.root)
        by_name = {entry['name']: entry for entry in listed}
        self.assertTrue(by_name[ready.name]['replayable'])
        for run, status in ((failed, 'failed'), (legacy, 'older sequence'),
                            (missing, 'missing files'), (broken, 'invalid recording')):
            self.assertEqual(by_name[run.name]['status'], status)
            self.assertFalse(by_name[run.name]['replayable'])
        self.assertEqual([entry['name'] for entry in listed if entry['name'] != broken.name],
                         [missing.name, legacy.name, failed.name, ready.name])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_recording_timestamp_uses_tokyo_date_and_time(self):
        run = create_run(self.root, datetime(2026, 11, 10, 16, 30, 45, tzinfo=timezone.utc))
        self.assertEqual(run.name, '2026-11-11_01-30-45')
        self.assertTrue((run / 'arm').is_dir())
        self.assertTrue((run / 'hand').is_dir())
        metadata = json.loads((run / 'run.json').read_text())
        self.assertEqual(metadata['created_at'], '2026-11-11T01:30:45+09:00')
        self.assertEqual(metadata['status'], 'recording')

    def test_grasp_names_preserve_existing_runs_and_latest_uses_creation_time(self):
        older = create_run(self.root, datetime(2026, 11, 11, 1, tzinfo=timezone.utc), grasp_file='captures/zebra.json')
        newer = create_run(self.root, datetime(2026, 11, 11, 2, tzinfo=timezone.utc), grasp_file='captures/grasps.json')
        repeated = create_run(self.root, datetime(2026, 11, 11, 3, tzinfo=timezone.utc), grasp_file='captures/grasps.json')
        self.assertEqual(newer.name, 'grasps')
        self.assertEqual(repeated.name, 'grasps_001')
        for run in (older, newer):
            finish_run(run, True)
        finish_run(repeated, False)
        self.assertEqual(resolve_run(self.root), newer)
        self.assertEqual(resolve_run(self.root, 'grasps'), newer)
        self.assertEqual([item['name'] for item in replay_run_choices(self.root)], ['grasps_001', 'grasps', 'zebra'])
        self.assertEqual(json.loads((newer / 'run.json').read_text())['grasp_file'], 'captures/grasps.json')

    def test_latest_capture_name_does_not_conflict_with_selection_keyword(self):
        run = create_run(self.root, grasp_file='latest.txt')
        self.assertEqual(run.name, 'latest_001')
        finish_run(run, True)
        self.assertEqual(resolve_run(self.root, run.name), run)

    def test_two_recordings_in_one_second_never_overwrite(self):
        now = datetime(2026, 11, 10, 4, 0, 0, tzinfo=timezone.utc)
        first = create_run(self.root, now)
        (first / 'arm/path.pkl').write_bytes(b'old recording')
        second = create_run(self.root, now)
        self.assertEqual(second.name, first.name + '_001')
        self.assertEqual((first / 'arm/path.pkl').read_bytes(), b'old recording')
        self.assertFalse((second / 'arm/path.pkl').exists())

    def test_latest_skips_in_progress_failed_and_malformed_runs(self):
        runs = [create_run(self.root, datetime(2026, 11, 10, hour, tzinfo=timezone.utc))
                for hour in range(5)]
        finish_run(runs[0], True)
        finish_run(runs[1], True)
        finish_run(runs[3], False)
        (runs[4] / 'run.json').write_text('broken JSON')
        self.assertEqual(resolve_run(self.root), runs[1])

    def test_selected_archive_and_explicit_run_resolve_to_same_recording(self):
        first, second = create_run(self.root), create_run(self.root)
        finish_run(first, True)
        finish_run(second, True)
        self.assertEqual(resolve_run(self.root, first.name), first)
        self.assertEqual(resolve_run(first), first)
        self.assertEqual(resolve_run(self.root), second)

    def test_incomplete_recording_is_not_replayable(self):
        run = create_run(self.root)
        with self.assertRaisesRegex(ValueError, 'No completed compatible trajectory'):
            resolve_run(self.root)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            resolve_run(run)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            resolve_run(self.root, run.name)

    def test_selected_name_cannot_escape_archive(self):
        for name in ('../other', '/tmp/other', 123, ''):
            with self.assertRaises(ValueError):
                resolve_run(self.root, name)

    def test_explicit_legacy_or_external_arm_hand_directory_still_works(self):
        for backend in ('arm', 'hand'):
            (self.root / backend).mkdir()
        self.assertEqual(resolve_run(self.root), self.root)

    def test_new_task_selection_skips_legacy_runs_and_rejects_explicit_legacy(self):
        legacy = create_run(self.root, datetime(2026, 11, 10, 1, tzinfo=timezone.utc))
        current = create_run(self.root, datetime(2026, 11, 10, 0, tzinfo=timezone.utc), task_version=2)
        finish_run(legacy, True)
        finish_run(current, True)
        self.assertEqual(resolve_run(self.root, task_version=2), current)
        with self.assertRaisesRegex(ValueError, 'old task'):
            resolve_run(self.root, legacy.name, task_version=2)


if __name__ == '__main__':
    unittest.main()
