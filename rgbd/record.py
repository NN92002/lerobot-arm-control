"""Record only selected arms and cameras; inactive devices never produce fields."""
import argparse
from contextlib import ExitStack
import copy
import hashlib
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
from .common import ROOT, MOTORS, load_config, state_vector, write_json
from .kinematics import ForwardKinematics
from .camera import RealSense


def active_sides(config):
    return tuple(config.get('active_sides', ('left', 'right')))


def select_config(config, sides=None, camera_names=None):
    result = copy.deepcopy(config)
    selected = set(active_sides(config) if sides is None else sides)
    if not selected <= {'left', 'right'}:
        raise ValueError('Unknown arm side')
    result['active_sides'] = [side for side in ('left', 'right') if side in selected]
    for role in ('robot', 'teleop'):
        result[role]['ports'] = {side: config[role]['ports'][side] for side in result['active_sides']}
    result['kinematics'] = {side: config['kinematics'][side] for side in result['active_sides']}
    if camera_names is not None:
        names = set(camera_names)
        if not names <= {c['name'] for c in config['cameras']}:
            raise ValueError('Unknown camera name')
        result['cameras'] = [c for c in result['cameras'] if c['name'] in names]
    return result


def layout_names(sides):
    joints = [f'{side}_{motor}' for side in sides for motor in MOTORS]
    states = joints + [f'{side}_ee_{name}' for side in sides
                       for name in ('x', 'y', 'z', 'r00', 'r10', 'r20', 'r01', 'r11', 'r21')]
    return joints, states


def disconnect(device):
    if device.is_connected:
        device.disconnect()


def make_arms(config):
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
    result = {}
    for role in ('robot', 'teleop'):
        for side in active_sides(config):
            common = dict(port=config[role]['ports'][side], id=f'{config[role]["id"]}_{side}',
                          calibration_dir=ROOT / 'calibration' / 'dual' / role, use_degrees=True)
            if role == 'robot':
                arm = SOFollower(SOFollowerRobotConfig(**common, max_relative_target=config['max_relative_target']))
            else:
                arm = SOLeader(SOLeaderTeleopConfig(**common))
            result[role, side] = arm
    return result


def validate_config(config, check_ports=True):
    if config['model'] not in ('so100', 'so101'):
        raise ValueError('Requires SO-100 or SO-101 arms')
    if type(config['fps']) is not int or not 1 <= config['fps'] <= 60:
        raise ValueError('FPS must be an integer from 1 to 60')
    limit = config['max_relative_target']
    if not np.isfinite(limit) or limit <= 0:
        raise ValueError('max_relative_target must be positive')
    seen = set()
    for side in active_sides(config):
        for role in ('robot', 'teleop'):
            port = Path(config[role]['ports'][side])
            if not port.is_absolute() or 'REPLACE' in str(port) or (check_ports and not port.exists()):
                raise ValueError(f'Configure the real {side} {role} port: {port}')
            identity = str(port.resolve())
            if identity in seen:
                raise ValueError('Each arm must use a different port')
            seen.add(identity)
    names, serials = set(), set()
    for camera in config['cameras']:
        name = camera['name']
        if not name.isidentifier() or name in names or camera['serial'] in serials:
            raise ValueError('Camera names and serials must be unique')
        if camera['fps'] < config['fps']:
            raise ValueError('Camera FPS must be >= recording FPS')
        names.add(name)
        serials.add(camera['serial'])


def build_solvers(config):
    solvers = {side: ForwardKinematics(config['kinematics'][side]) for side in active_sides(config)}
    for solver in solvers.values():
        solver.compute(dict.fromkeys(MOTORS, 0.0))
    return solvers


def validate_hardware(config):
    validate_config(config)
    if not active_sides(config) and not config['cameras']:
        raise ValueError('Select at least one arm pair or camera')
    return build_solvers(config)


def mock_cameras(config):
    return {c['name']: SimpleNamespace(metadata={'depth_scale_m': .001, 'synthetic': True,
                                                'depth_aligned_to': 'color'}) for c in config['cameras']}


def sample_frame(config, cameras, arms, solvers, mock=False, index=0, teleop_sides=None, cancel_event=None):
    sides = active_sides(config)
    enabled = set(sides if teleop_sides is None else teleop_sides)
    values = {}
    if mock:
        for camera in config['cameras']:
            h, w = camera['height'], camera['width']
            raw = np.full((h, w), 1000 + index % 1000, dtype=np.uint16)
            raw[0, 0] = 0
            values.update({f'{camera["name"]}__rgb': np.full((h, w, 3), index % 255, dtype=np.uint8),
                           f'{camera["name"]}__depth': raw,
                           f'{camera["name"]}__camera_host_time': np.float64(index / config['fps'])})
        sample_time = index / config['fps']
        joints = np.sin(index / 10 + np.arange(6 * len(sides))).astype(np.float32) * 10
        joints[5::6] = 50
        poses = np.repeat(np.eye(4, dtype=np.float32)[None], len(sides), axis=0)
        poses[:, 0, 3] = joints[::6] / 100
        requested = joints + .1
        action = requested.copy()
        span = 0.0
        sent_time = sample_time
    else:
        for name, camera in cameras.items():
            values.update({f'{name}__{key}': value for key, value in camera.read().items()})
        obs_start = time.monotonic()
        observations = {side: arms['robot', side].get_observation() for side in sides}
        sample_time = time.monotonic()
        span = sample_time - obs_start
        joints = np.asarray([observations[side][motor + '.pos'] for side in sides for motor in MOTORS], dtype=np.float32)
        poses = np.asarray([solvers[side].compute({m: observations[side][m + '.pos'] for m in MOTORS})
                            if side in solvers else np.full((4, 4), np.nan) for side in sides], dtype=np.float32).reshape(-1, 4, 4)
        requests, sent = {}, {}
        for side in sides:
            if cancel_event is not None and cancel_event.is_set():
                raise InterruptedError('Session is closing')
            if side in enabled:
                requests[side] = arms['teleop', side].get_action()
                sent[side] = arms['robot', side].send_action(requests[side])
        requested = np.asarray([requests[side][m + '.pos'] if side in enabled else np.nan
                                for side in sides for m in MOTORS], dtype=np.float32)
        action = np.asarray([sent[side][m + '.pos'] if side in enabled else np.nan
                             for side in sides for m in MOTORS], dtype=np.float32)
        sent_time = time.monotonic()
    values.update(host_time=np.float64(sample_time), observation_span_s=np.float64(span))
    if sides:
        # Idle arms have observations but no issued command. Never invent action labels.
        for i, side in enumerate(sides):
            if side not in enabled:
                requested[i*6:(i+1)*6] = np.nan
                action[i*6:(i+1)*6] = np.nan
        values.update(joints=joints, ee_transform=poses, state=state_vector(joints, poses), action=action,
                      requested_action=requested, action_valid=np.asarray([s in enabled for s in sides], dtype=bool),
                      action_host_time=np.float64(sent_time))
    return values


class EpisodeWriter:
    def __init__(self, directory, config, cameras, mock=False, teleop_sides=None):
        self.directory = Path(directory)
        self.timestamps = []
        sides = active_sides(config)
        joints, states = layout_names(sides)
        self.meta = dict(format_version=2, complete=False, synthetic=mock, fps=config['fps'], task=config['task'],
                         active_sides=list(sides), joint_names=joints, state_names=states,
                         teleop_sides=list(sides if teleop_sides is None else teleop_sides),
                         action_semantics='sent absolute targets after clamp; NaN with action_valid=false for idle arms',
                         joint_units='degrees for 5 body joints per arm; gripper normalized 0..100',
                         ee_units='metres and rotation matrix; FK estimate',
                         pose_frames={s: config['kinematics'][s]['output_frame'] for s in sides},
                         cameras={name: camera.metadata for name, camera in cameras.items()},
                         config=select_config(config), frames=0)
        if not mock:
            self.meta['urdf_sha256'] = {s: hashlib.sha256(Path(config['kinematics'][s]['urdf']).read_bytes()).hexdigest() for s in sides}
        self.directory.mkdir(parents=True, exist_ok=False)
        write_json(self.directory / 'episode.json', self.meta)

    def append(self, values):
        sides = self.meta['active_sides']
        if sides:
            if not np.isfinite(values['state']).all():
                raise ValueError('Nonfinite arm state; configure valid FK before recording')
            enabled = np.repeat(values['action_valid'], 6)
            if not np.isfinite(values['action'][enabled]).all():
                raise ValueError('Nonfinite issued targets')
        index = self.meta['frames']
        temp = self.directory / f'{index:06d}.npz.tmp'
        with temp.open('wb') as handle:
            np.savez(handle, **values)
        temp.replace(self.directory / f'{index:06d}.npz')
        self.timestamps.append(float(values['host_time']))
        self.meta['frames'] += 1

    def close(self, complete=False, reason=None):
        self.meta['complete'] = complete
        if reason:
            self.meta['stop_reason'] = reason
            self.meta['stopped_by_user'] = reason == 'user'
        if len(self.timestamps) > 1:
            span = self.timestamps[-1] - self.timestamps[0]
            self.meta['effective_fps'] = (len(self.timestamps)-1) / span if span > 0 else 0
            self.meta['max_frame_gap_s'] = float(np.diff(self.timestamps).max())
        write_json(self.directory / 'episode.json', self.meta)


def record_episode(directory, config, frames, cameras, arms, solvers, mock=False, on_frame=None, stop_event=None):
    writer = EpisodeWriter(directory, config, cameras, mock)
    started = time.monotonic()
    reason = None
    try:
        for index in range(frames):
            if stop_event is not None and stop_event.is_set():
                reason = 'user'
                break
            values = sample_frame(config, cameras, arms, solvers, mock, index)
            writer.append(values)
            if on_frame:
                on_frame(values, dict(writer.meta))
            if not mock or on_frame:
                time.sleep(max(0, started+(index+1)/config['fps']-time.monotonic()))
    except BaseException:
        reason = 'error'
        raise
    finally:
        writer.close(writer.meta['frames'] == frames and reason is None, reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episodes', type=int, default=1)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--arms', choices=('both', 'left', 'right', 'none'), default='both')
    parser.add_argument('--cameras', nargs='*', help='Selected camera names; use without names for arm-only recording')
    args = parser.parse_args()
    sides = ('left', 'right') if args.arms == 'both' else () if args.arms == 'none' else (args.arms,)
    cfg = select_config(load_config(args.config), sides, args.cameras)
    frames = int(args.seconds * cfg['fps'])
    if frames < 2 or args.episodes < 1 or not sides and not cfg['cameras']:
        parser.error('Need at least two frames, one episode, and one active device')
    solvers = {} if args.mock else validate_hardware(cfg)
    args.output.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        cameras, arms = {}, {}
        if args.mock:
            cameras = mock_cameras(cfg)
        else:
            for spec in cfg['cameras']:
                camera = RealSense(spec)
                stack.callback(camera.close)
                camera.connect()
                cameras[spec['name']] = camera
            arms = make_arms(cfg)
            for arm in arms.values():
                stack.callback(disconnect, arm)
                arm.connect(calibrate=False)
                if not arm.is_calibrated:
                    raise ValueError('Calibrate selected arms first')
        for _ in range(args.episodes):
            if not args.mock:
                input('Prepare the scene, then press Enter to record; Ctrl+C to stop: ')
            index = 0
            while (args.output / f'episode_{index:06d}').exists():
                index += 1
            record_episode(args.output / f'episode_{index:06d}', cfg, frames, cameras, arms, solvers, args.mock)
    print(f'Data saved: {args.output.resolve()}')


if __name__ == '__main__':
    main()
