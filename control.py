#!/usr/bin/env python3
"""Generate and run local LeRobot teleoperation commands."""
import argparse
import glob
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def build_command(config, operation, side=None):
    mode = config['mode']
    if side is not None and (side not in ('left', 'right') or mode != 'dual'):
        raise ValueError('--side requires a dual config and left or right')
    selected_side = side
    if mode not in ('single', 'dual'):
        raise ValueError('mode 必須是 single 或 dual')
    fps = config.get('fps', 30)
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError('fps 必須是 1 到 60 的整數')
    limit = config.get('max_relative_target', 5)
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit <= 0:
        raise ValueError('max_relative_target 必須是正數')
    model = config.get('model', 'so101')
    if model not in ('so100', 'so101'):
        raise ValueError('此專案設定支援 so100 / so101')
    module = 'lerobot_teleoperate' if operation == 'teleoperate' else 'lerobot_calibrate'
    executable = Path(config['python'])
    if not executable.is_absolute():
        executable = ROOT / executable
    command = [str(executable), '-m', 'lerobot.scripts.' + module]
    roles = ('robot', 'teleop') if operation == 'teleoperate' else (operation.removeprefix('calibrate-'),)
    ports = []
    for role in roles:
        suffix = 'follower' if role == 'robot' else 'leader'
        kind = 'bi_so_' + suffix if mode == 'dual' and selected_side is None else model + '_' + suffix
        device_id = config[role]['id'] + ('_' + selected_side if selected_side else '')
        command.extend([f'--{role}.type={kind}', f'--{role}.id={device_id}',
                        f'--{role}.calibration_dir={ROOT / "calibration" / mode / role}'])
        sides = (selected_side,) if selected_side else ('left', 'right') if mode == 'dual' else ('single',)
        for side in sides:
            port = config[role]['ports'][side]
            if not isinstance(port, str) or not port.startswith('/dev/'):
                raise ValueError('連接埠必須是 /dev/ 下的裝置路徑')
            ports.append(port)
            prefix = f'{role}.{side}_arm_config' if mode == 'dual' and selected_side is None else role
            command.append(f'--{prefix}.port={port}')
            if role == 'robot':
                command.append(f'--{prefix}.max_relative_target={limit}')
    if len(set(map(os.path.realpath, ports))) != len(ports):
        raise ValueError('每支手臂必須使用不同連接埠')
    if operation == 'teleoperate':
        command.extend([f'--fps={fps}', '--display_data=false'])
    return command, ports


def main():
    parser = argparse.ArgumentParser(description='SO 單組／雙組主從手臂控制。預設只預覽指令。')
    parser.add_argument('operation', choices=['ports', 'teleoperate', 'calibrate-robot', 'calibrate-teleop'])
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/dual.json')
    parser.add_argument('--side', choices=('left', 'right'), help='Only this pair in a dual configuration')
    parser.add_argument('--run', action='store_true', help='實際執行並連接硬體')
    args = parser.parse_args()
    if args.operation == 'ports':
        ports = sorted(set(glob.glob('/dev/serial/by-id/*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')))
        print('\n'.join(ports) if ports else '目前未找到 USB 串列手臂裝置。')
        return 0
    config = json.loads(args.config.read_text())
    command, ports = build_command(config, args.operation, args.side)
    source = Path(config['lerobot_path']) / 'src' if config.get('lerobot_path') else None
    if source is not None and not (source / 'lerobot').is_dir():
        raise ValueError(f'找不到 LeRobot 原始碼：{source}')
    print(shlex.join(command), flush=True)
    if not args.run:
        print('指令預覽完成；加上 --run 才會連接手臂。')
        return 0
    for port in ports:
        if not Path(port).exists():
            raise ValueError(f'連接埠不存在，請先修改設定：{port}')
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env.pop('PYTHONHOME', None)
    env['PYTHONNOUSERSITE'] = '1'
    if source is not None:
        env['PYTHONPATH'] = str(source)
    return subprocess.call(command, env=env)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError) as exc:
        print(f'錯誤：{exc}', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
