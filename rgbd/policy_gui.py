"""PyQt6 GUI for selecting and running a Diffusion Policy on follower arms."""
import argparse
import json
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import (QApplication, QComboBox, QGridLayout, QHBoxLayout,
                             QInputDialog, QLabel, QMainWindow, QMessageBox,
                             QPushButton, QVBoxLayout, QWidget)

from .camera import RealSense
from .common import ROOT, load_config
from .dataset import depth_view, normalize
from .record import build_solvers, select_config

MOTORS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
IDENTIFICATION_DEGREES = 2.0


def scan_ports():
    stable = sorted(path for path in Path('/dev/serial/by-id').glob('*') if path.exists())
    resolved = {path.resolve() for path in stable}
    fallback = sorted(path for pattern in ('ttyACM*', 'ttyUSB*') for path in Path('/dev').glob(pattern)
                      if path.resolve() not in resolved)
    return [str(path) for path in stable + fallback]


def local_checkpoints(root):
    root = Path(root)
    if not root.is_dir():
        return []
    return sorted(path for path in root.iterdir() if path.is_dir() and
                  (path / 'model' / 'checkpoint_best' / 'config.json').is_file())


def remote_checkpoints(server):
    ssh = server.get('ssh', 'itri2026@140.114.58.2')
    root = server.get('checkpoint_path', '/home/itri2026/lerobot_checkpoints')
    command = ['ssh', ssh, 'find', root, '-mindepth', '1', '-maxdepth', '1',
               '-type', 'd', '-printf', '%f\\n']
    result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=20)
    return sorted(name.strip() for name in result.stdout.splitlines() if name.strip())


def download_checkpoint(name, destination, server):
    if not name or Path(name).name != name:
        raise ValueError('Invalid checkpoint name')
    ssh = server.get('ssh', 'itri2026@140.114.58.2')
    root = server.get('checkpoint_path', '/home/itri2026/lerobot_checkpoints')
    destination = Path(destination) / name
    destination.mkdir(parents=True, exist_ok=True)
    subprocess.run(['rsync', '-av', '--partial', f'{ssh}:{root}/{name}/',
                    f'{destination}/'], check=True)
    if not (destination / 'model' / 'checkpoint_best' / 'config.json').is_file():
        raise ValueError(f'Downloaded checkpoint is incomplete: {destination}')
    return destination


class Runtime(QObject):
    status = pyqtSignal(str)
    frame = pyqtSignal(object)
    finished = pyqtSignal()

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.camera = None
        self.arms = {}
        self.solvers = {}
        self.policy = None
        self.stop_event = threading.Event()
        self.motion_event = threading.Event()
        self.thread = None

    def connect_camera(self):
        if self.camera is not None:
            return
        if len(self.config['cameras']) != 1:
            raise ValueError('Policy runtime requires exactly one configured camera')
        self.camera = RealSense(self.config['cameras'][0])
        self.camera.connect()
        self.status.emit('Camera connected')

    def connect_arm(self, side):
        if ('robot', side) in self.arms:
            return
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        arm = SOFollower(SOFollowerRobotConfig(
            port=self.config['robot']['ports'][side],
            id=f"{self.config['robot']['id']}_{side}",
            calibration_dir=ROOT / 'calibration' / 'dual' / 'robot',
            use_degrees=True,
            max_relative_target=float(self.config['max_relative_target'])))
        arm.connect(calibrate=False)
        if not arm.is_calibrated:
            arm.disconnect()
            raise ValueError(f'{side} follower is not calibrated')
        self.arms['robot', side] = arm
        self.solvers[side] = build_solvers(select_config(self.config, [side], []))[side]
        self.status.emit(f'{side.title()} follower connected')

    def identify_arm(self, side):
        arm = self.arms.get(('robot', side))
        if arm is None:
            raise ValueError(f'Connect the {side} follower first')
        original = float(arm.get_observation()['shoulder_pan.pos'])
        target = original + IDENTIFICATION_DEGREES
        try:
            arm.send_action({'shoulder_pan.pos': target})
            time.sleep(0.35)
        finally:
            arm.send_action({'shoulder_pan.pos': original})
        self.status.emit(f'{side.title()} follower identification move completed (+2 deg, returned)')

    def set_arm_port(self, side, port):
        if not port.startswith('/dev/'):
            raise ValueError(f'{side} follower port must be under /dev')
        self.config['robot']['ports'][side] = port

    def load_policy(self, checkpoint):
        metadata_path = Path(checkpoint) / 'model' / 'checkpoint_best' / 'preprocessing.json'
        if not metadata_path.is_file():
            raise ValueError(f'No checkpoint_best preprocessing metadata: {checkpoint}')
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
        self.metadata = json.loads(metadata_path.read_text())
        expected = self.metadata['joint_names']
        actual = [f'{side}_{motor}' for side in ('left', 'right') for motor in MOTORS]
        if expected != actual:
            raise ValueError('Checkpoint joint layout is not the configured dual-arm layout')
        self.policy = DiffusionPolicy.from_pretrained(
            str(Path(checkpoint) / 'model' / 'checkpoint_best'), local_files_only=True).cpu().eval()
        self.status.emit(f'Loaded {Path(checkpoint).name} / checkpoint_best')

    def start(self):
        if self.camera is None or len(self.arms) != 2:
            raise ValueError('Connect camera and both follower arms first')
        if self.policy is None:
            raise ValueError('Select and load a checkpoint first')
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.motion_event.set()
        self.policy.reset()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.status.emit('Policy running')

    def stop_motion(self):
        self.motion_event.clear()
        self.stop_event.set()
        self.status.emit('Policy stopped; arms remain connected')

    def emergency_stop(self):
        self.motion_event.clear()
        self.stop_event.set()
        self.close_hardware()
        self.status.emit('EMERGENCY STOP: policy stopped and hardware disconnected')

    def close_hardware(self):
        for arm in list(self.arms.values()):
            try:
                if arm.is_connected:
                    arm.disconnect()
            except Exception as error:
                self.status.emit(f'Arm disconnect failed: {error}')
        self.arms.clear()
        self.solvers.clear()
        if self.camera is not None:
            try:
                self.camera.close()
            finally:
                self.camera = None

    def disconnect_camera(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None
            self.status.emit('Camera disconnected')

    def disconnect_arm(self, side):
        arm = self.arms.pop(('robot', side), None)
        if arm is not None:
            if arm.is_connected:
                arm.disconnect()
            self.solvers.pop(side, None)
            self.status.emit(f'{side.title()} follower disconnected')

    def close(self):
        self.stop_event.set()
        self.motion_event.clear()
        if self.thread:
            self.thread.join(timeout=5)
        self.close_hardware()

    def _observation(self, frame):
        observations = {side: self.arms['robot', side].get_observation() for side in ('left', 'right')}
        joints = np.asarray([observations[side][motor + '.pos'] for side in ('left', 'right')
                             for motor in MOTORS], dtype=np.float32)
        poses = [self.solvers[side].compute({motor: observations[side][motor + '.pos'] for motor in MOTORS})
                 for side in ('left', 'right')]
        state = np.concatenate((joints, *[np.concatenate((pose[:3, 3], pose[:3, 0], pose[:3, 1]))
                                          for pose in poses])).astype(np.float32)
        size = self.metadata['image_size']
        rgb = cv2.resize(frame['rgb'], (size, size), interpolation=cv2.INTER_AREA)
        depth = depth_view(frame['depth'], self.camera.metadata['depth_scale_m'],
                           self.metadata['max_depth_m'], size)
        state = normalize(state, np.asarray(self.metadata['stats']['state_min']),
                          np.asarray(self.metadata['stats']['state_max']))
        return state, rgb.transpose(2, 0, 1).astype(np.float32) / 255, depth

    def loop(self):
        period = 1 / self.metadata['fps']
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                frame = self.camera.read()
                state, rgb, depth = self._observation(frame)
                batch = {
                    'observation.state': torch.from_numpy(state).unsqueeze(0),
                    'observation.images.front_rgb': torch.from_numpy(rgb).unsqueeze(0),
                    'observation.images.front_depth': torch.from_numpy(depth).unsqueeze(0),
                }
                with torch.no_grad():
                    action = self.policy.select_action(batch)[0].cpu().numpy()
                lower = np.asarray(self.metadata['stats']['action_min'])
                upper = np.asarray(self.metadata['stats']['action_max'])
                target = (action + 1) / 2 * np.maximum(upper - lower, 1e-6) + lower
                if not np.isfinite(target).all() or target.shape != (12,):
                    raise RuntimeError('Policy produced an invalid action')
                if self.motion_event.is_set():
                    for index, side in enumerate(('left', 'right')):
                        command = {motor + '.pos': float(target[index * 6 + offset]) for offset, motor in enumerate(MOTORS)}
                        self.arms['robot', side].send_action(command)
                self.frame.emit(frame['rgb'])
                self.stop_event.wait(max(0, started + period - time.monotonic()))
        except Exception as error:
            self.motion_event.clear()
            self.status.emit(f'Policy stopped by error: {type(error).__name__}: {error}')
        finally:
            self.finished.emit()


class PolicyUI(QMainWindow):
    def __init__(self, config_path, checkpoint_root):
        super().__init__()
        self.config_path = Path(config_path)
        self.config = load_config(config_path)
        self.checkpoint_root = Path(checkpoint_root)
        self.runtime = Runtime(self.config)
        self.runtime.status.connect(self.status)
        self.runtime.frame.connect(self.on_frame)
        self.runtime.finished.connect(lambda: self.run_button.setText('Start Policy'))
        self.setWindowTitle('LeRobot Studio | Policy Runner')
        self.resize(720, 480)
        self.build_ui()
        self.refresh_checkpoints()

    def build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        title = QLabel('DIFFUSION POLICY  /  HARDWARE RUNNER')
        title.setStyleSheet('font-size: 20px; font-weight: 700;')
        layout.addWidget(title)
        layout.addWidget(QLabel('Checkpoint selection and hardware controls'))
        grid = QGridLayout()
        self.checkpoint = QComboBox()
        refresh = QPushButton('Refresh')
        refresh.clicked.connect(self.refresh_checkpoints)
        download = QPushButton('Download selected remote')
        download.clicked.connect(self.download_selected)
        grid.addWidget(QLabel('Local checkpoint'), 0, 0)
        grid.addWidget(self.checkpoint, 0, 1)
        grid.addWidget(refresh, 0, 2)
        grid.addWidget(download, 1, 1, 1, 2)
        self.port_boxes = {}
        for row, side in enumerate(('left', 'right'), start=2):
            grid.addWidget(QLabel(f'{side.title()} follower port'), row, 0)
            box = QComboBox()
            box.setEditable(True)
            box.setCurrentText(self.config['robot']['ports'][side])
            box.setToolTip(self.config['robot']['ports'][side])
            self.port_boxes[side] = box
            grid.addWidget(box, row, 1, 1, 2)
        rescan = QPushButton('Rescan Ports')
        rescan.clicked.connect(self.refresh_ports)
        save_ports = QPushButton('Save Ports')
        save_ports.clicked.connect(self.save_ports)
        grid.addWidget(rescan, 4, 1)
        grid.addWidget(save_ports, 4, 2)
        layout.addLayout(grid)
        hardware = QHBoxLayout()
        self.camera_button = QPushButton('Start Camera')
        self.camera_button.clicked.connect(self.toggle_camera)
        hardware.addWidget(self.camera_button)
        self.arm_buttons = {}
        self.identify_buttons = {}
        for side in ('left', 'right'):
            button = QPushButton(f'Start {side.title()} Follower')
            button.clicked.connect(lambda checked=False, selected=side: self.toggle_arm(selected))
            self.arm_buttons[side] = button
            hardware.addWidget(button)
            identify = QPushButton(f'Test {side.title()} 2 deg')
            identify.setEnabled(False)
            identify.clicked.connect(lambda checked=False, selected=side: self.identify_arm(selected))
            self.identify_buttons[side] = identify
            hardware.addWidget(identify)
        layout.addLayout(hardware)
        actions = QHBoxLayout()
        load = QPushButton('Load Checkpoint')
        load.clicked.connect(self.load_selected)
        self.run_button = QPushButton('Start Policy')
        self.run_button.clicked.connect(self.toggle_policy)
        stop = QPushButton('Stop Policy')
        stop.clicked.connect(self.runtime.stop_motion)
        emergency = QPushButton('EMERGENCY STOP')
        emergency.setStyleSheet('background: #b42318; color: white; font-weight: 700;')
        emergency.clicked.connect(self.runtime.emergency_stop)
        for button in (load, self.run_button, stop, emergency):
            actions.addWidget(button)
        layout.addLayout(actions)
        self.preview = QLabel('No live camera frame')
        self.preview.setMinimumHeight(260)
        self.preview.setStyleSheet('background: #101828; color: #98a2b3; qproperty-alignment: AlignCenter;')
        layout.addWidget(self.preview, 1)
        self.status_label = QLabel('Ready. Connect hardware explicitly.')
        layout.addWidget(self.status_label)
        self.setCentralWidget(root)

    def status(self, text):
        self.status_label.setText(text)

    def on_frame(self, pixels):
        self.preview.setText(f'Live RGB frame: {pixels.shape[1]} x {pixels.shape[0]}')

    def refresh_checkpoints(self):
        self.checkpoint.clear()
        for path in local_checkpoints(self.checkpoint_root):
            self.checkpoint.addItem(path.name, str(path))
        try:
            names = remote_checkpoints(self.config.get('server', {}))
            self.status(f'Local checkpoints refreshed; remote: {", ".join(names)}')
        except Exception as error:
            self.status(f'Local checkpoints refreshed; remote list unavailable: {error}')

    def refresh_ports(self):
        ports = scan_ports()
        for box in self.port_boxes.values():
            current = box.currentText()
            box.clear()
            box.addItems(ports)
            box.setCurrentText(current)
        self.status(f'Found {len(ports)} serial port(s)')

    def save_ports(self):
        try:
            selected = {side: box.currentText().strip() for side, box in self.port_boxes.items()}
            if not all(selected.values()) or len(set(selected.values())) != 2:
                raise ValueError('Left and right follower ports must be different and non-empty')
            for side, port in selected.items():
                self.runtime.set_arm_port(side, port)
            self.config_path = Path(self.config_path)
            self.config_path.write_text(json.dumps(self.config, indent=2, ensure_ascii=False) + '\n')
            self.status('Follower ports saved')
        except Exception as error:
            QMessageBox.critical(self, 'Save ports failed', str(error))

    def download_selected(self):
        try:
            names = remote_checkpoints(self.config.get('server', {}))
            name, ok = QInputDialog.getItem(self, 'Remote checkpoint', 'Checkpoint:', names, 0, False)
            if not ok:
                return
            download_checkpoint(name, self.checkpoint_root, self.config.get('server', {}))
            self.refresh_checkpoints()
            self.checkpoint.setCurrentText(name)
        except Exception as error:
            QMessageBox.critical(self, 'Download failed', str(error))

    def load_selected(self):
        if not self.checkpoint.currentData():
            self.status('No local checkpoint selected')
            return
        try:
            self.runtime.load_policy(self.checkpoint.currentData())
        except Exception as error:
            QMessageBox.critical(self, 'Load failed', str(error))

    def toggle_camera(self):
        try:
            if self.runtime.camera is None:
                self.runtime.connect_camera()
                self.camera_button.setText('Camera Connected')
            else:
                self.runtime.disconnect_camera()
                self.camera_button.setText('Start Camera')
        except Exception as error:
            QMessageBox.critical(self, 'Camera error', str(error))

    def toggle_arm(self, side):
        connected_now = False
        try:
            if ('robot', side) not in self.runtime.arms:
                self.runtime.set_arm_port(side, self.port_boxes[side].currentText().strip())
                self.runtime.connect_arm(side)
                connected_now = True
                self.runtime.identify_arm(side)
                self.arm_buttons[side].setText(f'{side.title()} Connected')
                self.identify_buttons[side].setEnabled(True)
            else:
                self.runtime.disconnect_arm(side)
                self.arm_buttons[side].setText(f'Start {side.title()} Follower')
                self.identify_buttons[side].setEnabled(False)
        except Exception as error:
            if connected_now:
                self.runtime.disconnect_arm(side)
                self.arm_buttons[side].setText(f'Start {side.title()} Follower')
                self.identify_buttons[side].setEnabled(False)
            QMessageBox.critical(self, 'Arm error', str(error))

    def identify_arm(self, side):
        try:
            if self.runtime.thread and self.runtime.thread.is_alive():
                raise ValueError('Stop Policy before running the identification move')
            self.runtime.identify_arm(side)
        except Exception as error:
            QMessageBox.critical(self, 'Identification move failed', str(error))

    def toggle_policy(self):
        try:
            if self.runtime.thread and self.runtime.thread.is_alive():
                self.runtime.stop_motion()
                self.run_button.setText('Start Policy')
            else:
                self.runtime.start()
                self.run_button.setText('Stop Policy')
        except Exception as error:
            QMessageBox.critical(self, 'Policy error', str(error))

    def closeEvent(self, event):
        self.runtime.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    parser.add_argument('--checkpoint-root', default=str(ROOT / 'checkpoints'))
    args = parser.parse_args()
    app = QApplication([])
    ui = PolicyUI(args.config, args.checkpoint_root)
    ui.show()
    app.exec()


if __name__ == '__main__':
    main()