import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description='Inspect captured episode integrity and timing without hardware')
    parser.add_argument('data', type=Path)
    args = parser.parse_args()
    found = False
    for path in sorted(args.data.glob('episode_*/episode.json')):
        found = True
        meta = json.loads(path.read_text())
        files = sorted(path.parent.glob('*.npz'))
        print(f'{path.parent.name}: complete={meta["complete"]}, synthetic={meta["synthetic"]}, frames={len(files)}, fps={meta.get("effective_fps", 0):.2f}, max_gap={meta.get("max_frame_gap_s", 0):.4f}s')
        print(f'  arms={meta.get("active_sides", ["left", "right"])}, cameras={list(meta["cameras"])}')
        if files:
            with np.load(files[0], allow_pickle=False) as frame:
                for key in ('joints', 'state', 'action', 'ee_transform'):
                    if key in frame:
                        print(f'  {key}: {frame[key].shape} {frame[key].dtype}')
                for name, camera in meta['cameras'].items():
                    depth = frame[f'{name}__depth']
                    print(f'  {name}: RGB={frame[f"{name}__rgb"].shape}, depth={depth.dtype} {depth.shape}, scale={camera["depth_scale_m"]}m, valid={(depth > 0).mean():.1%}')
    if not found:
        parser.error('No episodes found')


if __name__ == '__main__':
    main()
