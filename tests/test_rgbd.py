import copy
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np
from rgbd.common import ROOT, MOTORS, state_vector
from rgbd.kinematics import ForwardKinematics
from rgbd.dataset import EpisodeDataset, depth_view
from rgbd.record import record_episode


class KinematicsTests(unittest.TestCase):
    def config(self):
        return dict(urdf=str(ROOT / 'tests/fixtures/two_joint.urdf'), base_link='base', tip_link='tip',
                    joint_map={'pan': {'motor': 'shoulder_pan'}})

    def test_known_quarter_turn(self):
        pose = ForwardKinematics(self.config()).compute({'shoulder_pan': 90})
        np.testing.assert_allclose(pose[:3, 3], [0, 1, 0], atol=1e-6)
        np.testing.assert_allclose(pose[:3, 0], [0, 1, 0], atol=1e-6)

    def test_base_transform(self):
        config = self.config()
        transform = np.eye(4)
        transform[0, 3] = 2
        config['world_from_base'] = transform.tolist()
        pose = ForwardKinematics(config).compute({'shoulder_pan': 0})
        np.testing.assert_allclose(pose[:3, 3], [3, 0, 0])

    def test_missing_joint_mapping_fails(self):
        config = self.config()
        config['joint_map'] = {}
        with self.assertRaises(ValueError):
            ForwardKinematics(config)

    def test_state_layout(self):
        poses = np.repeat(np.eye(4)[None], 2, axis=0)
        state = state_vector(np.arange(12), poses)
        self.assertEqual(state.shape, (30,))
        np.testing.assert_array_equal(state[12:21], [0, 0, 0, 1, 0, 0, 0, 1, 0])


class DataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        cfg = json.loads((ROOT / 'configs/rgbd.json').read_text())
        cfg['cameras'][0].update(width=32, height=32)
        self.cfg = cfg
        cameras = {'front': SimpleNamespace(metadata={'depth_scale_m': .001})}
        for index in range(2):
            record_episode(self.root / f'episode_{index:06d}', cfg, 12, cameras, {}, {}, mock=True)

    def test_depth_metric_encoding_and_invalid(self):
        raw = np.array([[0, 1000], [2000, 4000]], dtype=np.uint16)
        encoded = depth_view(raw, .001, 2, 2)
        np.testing.assert_allclose(encoded[0], [[0, .505], [1, 1]], atol=1e-6)
        np.testing.assert_array_equal(encoded[0], encoded[2])

    def test_windows_stay_within_episode_and_include_depth(self):
        ds = EpisodeDataset(self.root, horizon=8, image_size=32, allow_mock=True)
        self.assertGreater(len(ds), 0)
        for episode, picks in ds.samples:
            self.assertTrue(np.all(picks < 12))
            self.assertTrue(np.all(picks >= 0))
        batch = ds[0]
        self.assertEqual(tuple(batch['action'].shape), (8, 12))
        self.assertEqual(tuple(batch['observation.images.front_depth'].shape), (2, 3, 32, 32))
        with np.load(self.root / 'episode_000000/000000.npz') as raw:
            self.assertEqual(raw['front__depth'].dtype, np.uint16)
            self.assertEqual(raw['front__depth'][1, 1], 1000)

    def test_mock_requires_explicit_flag(self):
        with self.assertRaisesRegex(ValueError, 'Synthetic'):
            EpisodeDataset(self.root, horizon=8)

    def test_time_gap_rejected(self):
        path = self.root / 'episode_000000/000006.npz'
        with np.load(path) as frame:
            values = dict(frame)
        values['host_time'] = np.float64(20)
        np.savez(path, **values)
        with self.assertRaisesRegex(ValueError, 'timestamps'):
            EpisodeDataset(self.root, horizon=8, allow_mock=True)

    def test_interrupted_episode_skipped(self):
        path = self.root / 'episode_000000/episode.json'
        meta = json.loads(path.read_text())
        meta['complete'] = False
        path.write_text(json.dumps(meta))
        ds = EpisodeDataset(self.root, horizon=8, allow_mock=True)
        self.assertEqual(len(ds.episodes), 1)


if __name__ == '__main__':
    unittest.main()
