"""Offline checkpoint check. Predict joint targets from an episode; never connects arms."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from .common import ROOT, load_config, write_json
from .dataset import EpisodeDataset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-mock', action='store_true')
    args = parser.parse_args()
    load_config(args.config)
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    torch.set_num_threads(min(4, torch.get_num_threads()))
    prep = json.loads((args.checkpoint / 'preprocessing.json').read_text())
    ds = EpisodeDataset(args.data, n_obs=prep['n_obs_steps'], horizon=prep['horizon'], image_size=prep['image_size'],
                        max_depth_m=prep['max_depth_m'], stats=prep['stats'], allow_mock=args.allow_mock)
    if ds.joint_names != prep['joint_names'] or ds.state_names != prep['state_names']:
        raise ValueError('Data arm layout differs from checkpoint')
    if ds.camera_names != prep['camera_names'] or ds.fps != prep['fps'] or ds.pose_frames != prep['pose_frames']:
        raise ValueError('Data camera order, fps or coordinate frames differ from checkpoint')
    policy = DiffusionPolicy.from_pretrained(str(args.checkpoint), local_files_only=True).cpu().eval()
    batch = ds[0]
    observations = {k: value.unsqueeze(0) for k, value in batch.items() if k.startswith('observation.')}
    # DiffusionModel.generate_actions consumes a stacked history directly.
    observations['observation.images'] = torch.stack([observations[key] for key in policy.config.image_features], dim=-4)
    with torch.no_grad():
        predicted = policy.diffusion.generate_actions(observations).cpu().numpy()[0]
    lower, upper = np.asarray(prep['stats']['action_min']), np.asarray(prep['stats']['action_max'])
    targets = (predicted + 1) / 2 * np.maximum(upper - lower, 1e-6) + lower
    if not np.isfinite(targets).all() or targets.shape[-1] != len(prep['joint_names']):
        raise RuntimeError('Invalid predicted action chunk')
    if args.output.exists():
        raise ValueError('Output already exists')
    write_json(args.output, dict(offline_only=True, joint_names=prep['joint_names'], units='body degrees; gripper 0..100', targets=targets.tolist()))
    print(f'Loaded checkpoint and predicted {len(targets)} actions: {args.output}; no hardware commands sent')


if __name__ == '__main__':
    main()
