import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from rgbd.common import ROOT
from rgbd.record import select_config, record_episode, mock_cameras, sample_frame
from rgbd.dataset import EpisodeDataset
from rgbd.session import HardwareSession


def config():
    cfg = json.loads((ROOT / 'configs/rgbd.json').read_text())
    cfg['cameras'][0].update(width=32, height=32)
    return cfg


class LayoutTests(unittest.TestCase):
    def test_device_combinations_have_only_selected_data(self):
        for sides, names in ((['left'], []), (['right'], ['front']), (['left', 'right'], ['front']), ([], ['front'])):
            with self.subTest(sides=sides, cameras=names), tempfile.TemporaryDirectory() as tmp:
                cfg = select_config(config(), sides, names)
                episode = Path(tmp) / 'episode_000000'
                record_episode(episode, cfg, 12, mock_cameras(cfg), {}, {}, mock=True)
                meta = json.loads((episode / 'episode.json').read_text())
                self.assertEqual(meta['active_sides'], sides)
                self.assertEqual(list(meta['cameras']), names)
                self.assertEqual(list(meta['config']['robot']['ports']), sides)
                with np.load(episode / '000000.npz') as frame:
                    self.assertEqual('front__rgb' in frame, bool(names))
                    self.assertEqual('joints' in frame, bool(sides))
                    if sides:
                        self.assertEqual(frame['joints'].shape, (len(sides)*6,))
                        self.assertEqual(frame['state'].shape, (len(sides)*15,))
                if sides:
                    dataset = EpisodeDataset(tmp, horizon=8, allow_mock=True)
                    self.assertEqual(dataset.action_dim, len(sides)*6)
                else:
                    with self.assertRaisesRegex(ValueError, 'Camera-only'):
                        EpisodeDataset(tmp, horizon=8, allow_mock=True)

    def test_left_and_right_datasets_cannot_mix(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i, side in enumerate(('left', 'right')):
                cfg = select_config(config(), [side], ['front'])
                record_episode(Path(tmp)/f'episode_{i:06d}', cfg, 12, mock_cameras(cfg), {}, {}, mock=True)
            with self.assertRaisesRegex(ValueError, 'Episodes must share'):
                EpisodeDataset(tmp, horizon=8, allow_mock=True)

    def test_idle_arm_has_no_invented_action(self):
        cfg = select_config(config(), ['left'], [])
        frame = sample_frame(cfg, {}, {}, {}, mock=True, teleop_sides=[])
        self.assertTrue(np.isfinite(frame['joints']).all())
        self.assertFalse(frame['action_valid'][0])
        self.assertTrue(np.isnan(frame['action']).all())


class FakeArm:
    def __init__(self, fail=False):
        self.is_connected = False
        self.is_calibrated = True
        self.fail = fail
        self.disconnects = self.sends = 0

    def connect(self, **kwargs):
        self.is_connected = True
        if self.fail:
            raise RuntimeError('Connect failed')

    def disconnect(self):
        self.disconnects += 1
        self.is_connected = False

    def get_observation(self):
        from rgbd.common import MOTORS
        return {m+'.pos': 0 for m in MOTORS}

    def get_action(self):
        return self.get_observation()

    def send_action(self, action):
        self.sends += 1
        return action


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.session = HardwareSession(config())
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.session.shutdown.set()
        self.session.thread.join(5)
        self.assertFalse(self.session.thread.is_alive())

    def command(self, operation, **args):
        self.session.submit(operation, **args)
        messages = []
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            event, value = self.session.events.get(timeout=5)
            messages.append((event, value))
            if event == 'command_done':
                return messages
        self.fail('Command timed out')

    def test_partial_connection_failure_cleans_both_arms(self):
        follower, leader = FakeArm(), FakeArm(fail=True)
        with patch('rgbd.session.validate_config'), patch('rgbd.session.make_arms', return_value={('robot','left'):follower, ('teleop','left'):leader}):
            self.command('connect', kind='arm', name='left', config=config(), mock=False)
        self.assertEqual(follower.disconnects, 1)
        self.assertEqual(leader.disconnects, 1)
        self.assertFalse(self.session.sides)

    def test_connect_does_not_send_targets_and_only_selected_pair_is_used(self):
        follower, leader = FakeArm(), FakeArm()
        with patch('rgbd.session.validate_config'), patch('rgbd.session.build_solvers', return_value={}), patch('rgbd.session.make_arms', return_value={('robot','right'):follower, ('teleop','right'):leader}) as factory:
            self.command('connect', kind='arm', name='right', config=config(), mock=False)
            self.session.frames.get(timeout=5)
            self.assertEqual(factory.call_args.args[0]['active_sides'], ['right'])
            self.assertEqual(follower.sends, 0)
            self.command('teleop', side='right', enabled=True)
            deadline = time.monotonic()+2
            while not follower.sends and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertGreater(follower.sends, 0)
            self.command('teleop', side='right', enabled=False)
            count = follower.sends
            time.sleep(.1)
            self.assertEqual(follower.sends, count)
            self.command('disconnect', kind='arm', name='right')
            self.assertEqual(follower.disconnects, 1)

    def test_recording_locks_hardware_selection(self):
        self.command('connect', kind='camera', name='front', config=config(), mock=True)
        with tempfile.TemporaryDirectory() as tmp:
            self.command('record', output=tmp, seconds=10)
            messages = self.command('connect', kind='arm', name='left', config=config(), mock=True)
            self.assertTrue(any('Stop recording' in str(value) for _, value in messages))
            self.assertFalse(self.session.sides)
            self.session.stop_recording.set()
            deadline = time.monotonic()+5
            while self.session.writer is not None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNone(self.session.writer)
            self.assertIn('front', self.session.cameras)
            meta = json.loads((Path(tmp)/'episode_000000/episode.json').read_text())
            self.assertFalse(meta['complete'])
