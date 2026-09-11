"""Episode-local timestamp windows for LeRobot's DiffusionPolicy.

Raw recordings remain lossless NPZ; this adapter supplies PyTorch batches without
claiming that the on-disk format is the standard LeRobotDataset format.
"""
import json
from pathlib import Path
import cv2
import numpy as np
import torch
from .record import layout_names


def nearest_indices(times, queries):
    right = np.searchsorted(times, queries).clip(0, len(times) - 1)
    left = (right - 1).clip(0, len(times) - 1)
    return np.where(np.abs(times[left] - queries) <= np.abs(times[right] - queries), left, right)


def normalize(values, lower, upper):
    return (2 * (values - lower) / np.maximum(upper - lower, 1e-6) - 1).astype(np.float32)


def depth_view(raw, scale, max_depth_m, size):
    # Fixed metric scale, never per-frame min/max or a coloured depth visualization.
    resized = cv2.resize(raw, (size, size), interpolation=cv2.INTER_NEAREST)
    depth = resized.astype(np.float32) * scale
    encoded = np.where(resized > 0, .01 + .99 * np.clip(depth / max_depth_m, 0, 1), 0)
    return np.repeat(encoded[None], 3, axis=0).astype(np.float32)


class EpisodeDataset(torch.utils.data.Dataset):
    def __init__(self, root, n_obs=2, horizon=16, image_size=96, max_depth_m=3.0, allow_mock=False, stats=None):
        self.root = Path(root)
        self.n_obs, self.horizon = n_obs, horizon
        self.image_size, self.max_depth_m = image_size, max_depth_m
        if image_size < 32 or not np.isfinite(max_depth_m) or max_depth_m <= 0:
            raise ValueError('image_size >= 32 and positive max_depth_m are required')
        self.episodes, self.samples = [], []
        state_min = state_max = action_min = action_max = None
        signature = None
        for path in sorted(self.root.glob('episode_*/episode.json')):
            meta = json.loads(path.read_text())
            if not meta['complete']:
                print(f'Skipping incomplete episode: {path.parent.name}')
                continue
            if meta['synthetic'] and not allow_mock:
                raise ValueError('Synthetic data requires --allow-mock; never mix with real demonstrations')
            sides = meta.get('active_sides', ['left', 'right'])
            if not sides:
                raise ValueError('Camera-only recordings have no action targets for policy training')
            expected_joints, expected_states = layout_names(sides)
            if meta['joint_names'] != expected_joints or meta['state_names'] != expected_states:
                raise ValueError('Unexpected joint/state layout')
            if set(meta.get('teleop_sides', sides)) != set(sides):
                raise ValueError('Read-only arm recordings have no command labels; enable teleop for training data')
            self.joint_names, self.state_names = expected_joints, expected_states
            self.action_dim, self.state_dim = len(expected_joints), len(expected_states)
            if state_min is None:
                state_min = np.full(self.state_dim, np.inf)
                state_max = -state_min.copy()
                action_min, action_max = np.full(self.action_dim, np.inf), np.full(self.action_dim, -np.inf)
            current = (meta['fps'], meta['joint_names'], meta['state_names'], meta['pose_frames'],
                       list(meta['cameras']), meta['synthetic'], meta.get('urdf_sha256'), meta['config']['kinematics'])
            if signature is not None and signature != current:
                raise ValueError('Episodes must share fps, joints, frames, cameras, URDF and real/synthetic type')
            signature = current
            self.camera_names = list(meta['cameras'])
            self.fps = meta['fps']
            self.pose_frames = meta['pose_frames']
            files = sorted(path.parent.glob('*.npz'))
            if len(files) != meta['frames'] or len(files) < horizon:
                raise ValueError(f'{path.parent}: frame count mismatch or fewer than horizon={horizon}')
            times, actions = [], []
            for file in files:
                with np.load(file, allow_pickle=False) as frame:
                    state, action = frame['state'], frame['action']
                    if state.shape != (self.state_dim,) or action.shape != (self.action_dim,) or not np.isfinite(state).all() or not np.isfinite(action).all():
                        raise ValueError(f'Invalid state/action: {file}')
                    for camera in self.camera_names:
                        rgb, depth = frame[f'{camera}__rgb'], frame[f'{camera}__depth']
                        if rgb.dtype != np.uint8 or depth.dtype != np.uint16 or rgb.shape != (*depth.shape, 3):
                            raise ValueError(f'Invalid RGB-D types/shapes: {file}')
                    times.append(float(frame['host_time']))
                    actions.append(action)
                    state_min, state_max = np.minimum(state_min, state), np.maximum(state_max, state)
                    action_min, action_max = np.minimum(action_min, action), np.maximum(action_max, action)
            times = np.asarray(times)
            gaps = np.diff(times)
            if not np.isfinite(times).all() or np.any(gaps <= 0) or np.any(gaps > 1.75 / self.fps):
                raise ValueError(f'{path.parent}: nonmonotonic timestamps or recording gaps > 1.75 frames; lower capture fps and re-record')
            episode_index = len(self.episodes)
            self.episodes.append((files, meta, times, np.asarray(actions)))
            for index in range(n_obs - 1, len(files)):
                query = times[index] + np.arange(1 - n_obs, 1 - n_obs + horizon) / self.fps
                if query[0] < times[0] - 1e-6 or query[-1] > times[-1] + 1e-6:
                    continue
                picks = nearest_indices(times, query)
                if np.max(np.abs(times[picks] - query)) <= .75 / self.fps:
                    self.samples.append((episode_index, picks))
        if not self.samples:
            raise ValueError('No valid complete episode windows found')
        self.stats = stats or dict(state_min=state_min.tolist(), state_max=state_max.tolist(),
                                   action_min=action_min.tolist(), action_max=action_max.tolist())

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        episode, picks = self.samples[index]
        files, meta, _, actions = self.episodes[episode]
        states = []
        views = {f'observation.images.{name}_{kind}': [] for name in self.camera_names for kind in ('rgb', 'depth')}
        for frame_index in picks[:self.n_obs]:
            with np.load(files[frame_index], allow_pickle=False) as frame:
                states.append(frame['state'])
                for camera in self.camera_names:
                    rgb = cv2.resize(frame[f'{camera}__rgb'], (self.image_size, self.image_size), interpolation=cv2.INTER_AREA)
                    views[f'observation.images.{camera}_rgb'].append(rgb.transpose(2, 0, 1).astype(np.float32) / 255)
                    views[f'observation.images.{camera}_depth'].append(depth_view(frame[f'{camera}__depth'], meta['cameras'][camera]['depth_scale_m'], self.max_depth_m, self.image_size))
        batch = {key: torch.from_numpy(np.stack(value)) for key, value in views.items()}
        batch['observation.state'] = torch.from_numpy(normalize(np.stack(states), np.asarray(self.stats['state_min']), np.asarray(self.stats['state_max'])))
        batch['action'] = torch.from_numpy(normalize(actions[picks], np.asarray(self.stats['action_min']), np.asarray(self.stats['action_max'])))
        batch['action_is_pad'] = torch.zeros(self.horizon, dtype=torch.bool)
        return batch
