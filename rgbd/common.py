import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MOTORS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
JOINT_NAMES = [f'{side}_{motor}' for side in ('left', 'right') for motor in MOTORS]
STATE_NAMES = JOINT_NAMES + [f'{side}_ee_{name}' for side in ('left', 'right')
                             for name in ('x', 'y', 'z', 'r00', 'r10', 'r20', 'r01', 'r11', 'r21')]


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    if cfg.get('lerobot_path'):
        sys.path.insert(0, str(Path(cfg['lerobot_path']) / 'src'))
    return cfg


def state_vector(joints, poses):
    """Joint degrees (gripper 0..100), XYZ metres, first two rotation columns."""
    ee = [np.concatenate((t[:3, 3], t[:3, 0], t[:3, 1])) for t in poses]
    return np.concatenate((joints, *ee)).astype(np.float32)


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix('.json.tmp')
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    temp.replace(path)
