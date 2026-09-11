"""Train the installed LeRobot DiffusionPolicy on lossless dual-arm RGB-D episodes."""
import argparse
import json
from pathlib import Path
import random
import numpy as np
import torch
from .common import ROOT, load_config, write_json
from .dataset import EpisodeDataset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=20000)
    parser.add_argument('--batch-size', type=int, default=8)
    parser.add_argument('--image-size', type=int, default=96)
    parser.add_argument('--max-depth-m', type=float, default=3)
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--allow-mock', action='store_true')
    parser.add_argument('--small', action='store_true', help='Small temporal U-Net for pipeline checks')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--save-every', type=int, default=1000)
    args = parser.parse_args()
    load_config(args.config)
    if args.steps < 1 or args.batch_size < 1 or args.save_every < 1:
        parser.error('steps, batch-size and save-every must be positive')
    if args.output.exists():
        parser.error('output already exists; choose a new checkpoint directory')
    torch.set_num_threads(min(4, torch.get_num_threads()))
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    from lerobot.configs.types import FeatureType, PolicyFeature, NormalizationMode
    from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    horizon = 8 if args.small else 16
    dataset = EpisodeDataset(args.data, horizon=horizon, image_size=args.image_size,
                             max_depth_m=args.max_depth_m, allow_mock=args.allow_mock)
    if not dataset.camera_names:
        raise ValueError('This image-conditioned Diffusion Policy requires a camera. Arm-only recordings can still be inspected.')
    inputs = {'observation.state': PolicyFeature(type=FeatureType.STATE, shape=(dataset.state_dim,))}
    inputs.update({f'observation.images.{camera}_{kind}': PolicyFeature(type=FeatureType.VISUAL, shape=(3, args.image_size, args.image_size))
                   for camera in dataset.camera_names for kind in ('rgb', 'depth')})
    cfg = DiffusionConfig(input_features=inputs, output_features={'action': PolicyFeature(type=FeatureType.ACTION, shape=(dataset.action_dim,))},
                          device=args.device, horizon=horizon, n_action_steps=4 if args.small else 8,
                          down_dims=(32, 64) if args.small else (256, 512, 1024),
                          crop_shape=None, pretrained_backbone_weights=None,
                          use_separate_rgb_encoder_per_camera=True, do_mask_loss_for_padding=True,
                          normalization_mapping={k: NormalizationMode.IDENTITY for k in ('VISUAL', 'STATE', 'ACTION')})
    policy = DiffusionPolicy(cfg).to(args.device)
    optimizer = cfg.get_optimizer_preset().build(policy.parameters())
    loader = torch.utils.data.DataLoader(dataset, batch_size=args.batch_size, shuffle=True, drop_last=False, num_workers=0)
    args.output.mkdir(parents=True, exist_ok=False)
    preprocessing = dict(format_version=1, stats=dataset.stats, state_names=dataset.state_names, joint_names=dataset.joint_names,
                         fps=dataset.fps, n_obs_steps=cfg.n_obs_steps, horizon=cfg.horizon,
                         image_size=args.image_size, max_depth_m=args.max_depth_m,
                         camera_names=dataset.camera_names, pose_frames=dataset.pose_frames,
                         rgb='uint8 -> float32 /255; area resize; no crop',
                         depth='raw uint16 * scale_m; nearest resize; valid=0.01+0.99*clip(m/max_depth_m,0,1); invalid=0; repeat 3 channels',
                         state_action='2*(x-min)/max(max-min,1e-6)-1; inverse action=(x+1)/2*(max-min)+min',
                         synthetic=dataset.episodes[0][1]['synthetic'],
                         hardware_config=dataset.episodes[0][1]['config'])
    write_json(args.output / 'preprocessing.json', preprocessing)
    write_json(args.output / 'training.json', {**vars(args), 'data': str(args.data.resolve()), 'output': str(args.output.resolve()), 'windows': len(dataset)})
    print(f'{len(dataset.episodes)} episodes, {len(dataset)} windows; RGB and depth keys: {list(inputs)[1:]}', flush=True)
    policy.train()
    iterator = iter(loader)
    for step in range(1, args.steps + 1):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            batch = next(iterator)
        batch = {key: value.to(args.device) for key, value in batch.items()}
        optimizer.zero_grad(set_to_none=True)
        loss, _ = policy(batch)
        if not torch.isfinite(loss):
            raise RuntimeError('Nonfinite training loss')
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0, error_if_nonfinite=True)
        optimizer.step()
        if step == 1 or step % 10 == 0 or step == args.steps:
            print(f'step={step}/{args.steps} loss={loss.item():.6f}', flush=True)
        if step % args.save_every == 0 or step == args.steps:
            checkpoint = args.output / f'checkpoint_{step:06d}'
            policy.save_pretrained(checkpoint)
            write_json(checkpoint / 'preprocessing.json', preprocessing)
            torch.save({'step': step, 'optimizer': optimizer.state_dict(), 'loss': loss.item()}, checkpoint / 'training_state.pt')
    print(f'Training complete: {args.output.resolve()}', flush=True)


if __name__ == '__main__':
    main()
