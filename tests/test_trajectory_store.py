"""Host-only checks for recording isolation, timestamps and replay selection."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'app'))
from trajectory_store import create_run, finish_run, resolve_run


class TrajectoryStoreTest(unittest.TestCase):
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
        with self.assertRaisesRegex(ValueError, 'No completed trajectory'):
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


if __name__ == '__main__':
    unittest.main()
