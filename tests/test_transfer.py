import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rgbd.transfer import next_remote_episode_index, upload_episode


class TransferTests(unittest.TestCase):
    def test_next_remote_episode_index(self):
        result = type('Result', (), {'stdout': 'episode_000000\nepisode_000004\nlegacy\n'})()
        with patch('rgbd.transfer.subprocess.run', return_value=result) as run:
            index = next_remote_episode_index({
                'ssh': 'itri2026@140.114.58.2',
                'dataset_path': '/home/itri2026/lerobot_datasets',
            }, 'pick_place')
        self.assertEqual(index, 5)
        self.assertIn('mkdir -p', run.call_args.args[0][2])

    def run_upload(self, remote_manifest):
        with tempfile.TemporaryDirectory() as tmp:
            episode = Path(tmp) / 'pick_place' / 'episode_000000'
            episode.mkdir(parents=True)
            (episode / 'episode.json').write_text('{}')
            (episode / '000000.npz').write_bytes(b'frame')

            def run(command, **kwargs):
                if command[0] == 'ssh' and 'find . ' in command[2]:
                    return type('Result', (), {'stdout': remote_manifest})()
                return type('Result', (), {})()

            with patch('rgbd.transfer.subprocess.run', side_effect=run):
                return upload_episode(episode, {
                    'ssh': 'itri2026@140.114.58.2',
                    'dataset_path': '/home/itri2026/lerobot_datasets',
                })

    def test_matching_remote_manifest_is_accepted(self):
        remote = '000000.npz\t5\nepisode.json\t2\n'
        result = self.run_upload(remote)
        self.assertEqual(result, '/home/itri2026/lerobot_datasets/pick_place/episode_000000')

    def test_mismatched_remote_manifest_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'verification failed'):
            self.run_upload('episode.json\t2\n')