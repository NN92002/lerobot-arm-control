"""PyQt6 recording studio. Run with ./run.sh -m rgbd.gui."""
import argparse
from pathlib import Path
import queue
import sys
import time

import numpy as np
from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QScrollArea, QSizePolicy,
    QSplitter, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from .common import ROOT, MOTORS, load_config, write_json
from .session import HardwareSession
ARM_PORTS = (('teleop', 'left', 'Left Leader'), ('robot', 'left', 'Left Follower'),
             ('teleop', 'right', 'Right Leader'), ('robot', 'right', 'Right Follower'))


def scan_ports():
    # Prefer stable USB names; avoid showing their tty aliases a second time.
    stable = sorted(p for p in Path('/dev/serial/by-id').glob('*') if p.exists())
    resolved = {p.resolve() for p in stable}
    fallback = sorted(p for pattern in ('ttyACM*', 'ttyUSB*') for p in Path('/dev').glob(pattern)
                      if p.resolve() not in resolved)
    return [str(p) for p in stable + fallback]


def apply_ports(config, selections):
    seen = {}
    for role, side, label in ARM_PORTS:
        port = selections[role, side].strip()
        if not port or 'REPLACE' in port:
            continue
        if not Path(port).is_absolute():
            raise ValueError(f'{label}: enter an absolute port path, e.g. /dev/ttyACM0')
        identity = str(Path(port).resolve())
        if identity in seen:
            raise ValueError(f'{label} and {seen[identity]} use the same port')
        seen[identity] = label
    for role, side, _ in ARM_PORTS:
        config[role]['ports'][side] = selections[role, side].strip()
    return config


def label(text, name=None, wrap=False):
    widget = QLabel(text)
    if name:
        widget.setObjectName(name)
    widget.setWordWrap(wrap)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    return widget


def card(title, subtitle=None):
    widget = QFrame()
    widget.setObjectName('card')
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(18, 16, 18, 16)
    layout.setSpacing(12)
    layout.addWidget(label(title, 'sectionTitle'))
    if subtitle:
        layout.addWidget(label(subtitle, 'muted', True))
    return widget, layout


class ImageView(QWidget):
    """Paint only on the GUI thread, keeping image ownership and aspect ratio."""
    def __init__(self):
        super().__init__()
        self.image = QImage()
        self.setMinimumSize(160, 90)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_frame(self, pixels):
        pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
        h, w, _ = pixels.shape
        self.image = QImage(pixels.data, w, h, pixels.strides[0], QImage.Format.Format_RGB888).copy()
        self.update()

    def clear(self):
        self.image = QImage()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#0b111c'))
        if self.image.isNull():
            painter.setPen(QPen(QColor('#182336'), 1))
            for x in range(0, self.width(), 32):
                painter.drawLine(x, 0, x, self.height())
            for y in range(0, self.height(), 32):
                painter.drawLine(0, y, self.width(), y)
            painter.setPen(QColor('#8b9bb2'))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, 'Camera disconnected\nConnect to start preview')
        else:
            size = self.image.size().scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio)
            target = QRectF((self.width()-size.width())/2, (self.height()-size.height())/2,
                            size.width(), size.height())
            painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
            painter.drawImage(target, self.image)
        painter.end()


class RecorderUI(QMainWindow):
    def __init__(self, config, output, mock=False):
        super().__init__()
        self.config_path = str(config)
        self.closing = False
        self.pending = False
        self.countdown_remaining = 0
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.countdown_tick)
        self.detected_ports = None
        self.last_frame_at = None
        self.depth_stats = []
        self.device_state = dict(sides=[], cameras=[], teleop=[], recording=False)
        self.setWindowTitle('LeRobot Studio | RGB-D Recorder')
        self.resize(1400, 940)
        self.setMinimumSize(960, 680)
        self.setFont(QFont('DejaVu Sans', 12))
        self.setStyleSheet(Path(__file__).with_name('gui.qss').read_text())
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 16)
        layout.setSpacing(18)
        header = QHBoxLayout()
        brand = QVBoxLayout()
        brand.addWidget(label('LEROBOT  /  STUDIO', 'eyebrow'))
        brand.addWidget(label('Record. Observe. Refine.', 'pageTitle'))
        header.addLayout(brand)
        header.addStretch()
        self.mode_badge = label('MOCK MODE' if mock else 'HARDWARE MODE', 'badge')
        header.addWidget(self.mode_badge)
        layout.addLayout(header)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        layout.addWidget(splitter, 1)

        sidebar = QWidget()
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(0, 0, 8, 0)
        side.setSpacing(16)
        arm_card, arm_layout = card('Arm connections', 'Two SO-101 leader / follower pairs')
        arm_layout.setSpacing(8)
        cfg = load_config(config)
        self.port_boxes = {}
        for role, arm_side, title in ARM_PORTS:
            arm_layout.addWidget(label(title, 'fieldLabel'))
            box = QComboBox()
            box.setEditable(True)
            box.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
            box.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            box.setMinimumContentsLength(12)
            box.setCurrentText(cfg[role]['ports'][arm_side])
            box.setToolTip(cfg[role]['ports'][arm_side])
            box.currentTextChanged.connect(box.setToolTip)
            arm_layout.addWidget(box)
            self.port_boxes[role, arm_side] = box
        row = QHBoxLayout()
        self.scan_button = QPushButton('Rescan Ports')
        self.scan_button.clicked.connect(self.refresh_ports)
        self.save_button = QPushButton('Save Ports')
        self.save_button.clicked.connect(self.save_ports)
        row.addWidget(self.scan_button)
        row.addWidget(self.save_button)
        arm_layout.addLayout(row)
        self.port_status = label('', 'muted', True)
        arm_layout.addWidget(self.port_status)
        self.arm_buttons, self.teleop_buttons, self.arm_status = {}, {}, {}
        for arm_side in ('left', 'right'):
            arm_layout.addWidget(label(f'{arm_side.title()} pair', 'fieldLabel'))
            controls = QHBoxLayout()
            connect = QPushButton('Connect')
            connect.clicked.connect(lambda checked=False, name=arm_side: self.toggle_device('arm', name))
            teleop = QPushButton('Start Teleop')
            teleop.clicked.connect(lambda checked=False, name=arm_side: self.toggle_teleop(name))
            controls.addWidget(connect)
            controls.addWidget(teleop)
            arm_layout.addLayout(controls)
            status = label('Disconnected', 'muted')
            arm_layout.addWidget(status)
            self.arm_buttons[arm_side] = connect
            self.teleop_buttons[arm_side] = teleop
            self.arm_status[arm_side] = status
        side.addWidget(arm_card)
        cameras_card, cameras_layout = card('Camera connections')
        self.camera_buttons = {}
        for spec in cfg['cameras']:
            row = QHBoxLayout()
            row.addWidget(label(spec['name']))
            button = QPushButton('Connect')
            button.clicked.connect(lambda checked=False, name=spec['name']: self.toggle_device('camera', name))
            row.addWidget(button)
            self.camera_buttons[spec['name']] = button
            cameras_layout.addLayout(row)
        side.insertWidget(0, cameras_card)
        record_card, settings = card('Recording setup')
        settings.addWidget(label('Output folder', 'fieldLabel'))
        self.path_entry = QLineEdit(str(output))
        self.path_entry.setToolTip(str(output))
        self.path_entry.textChanged.connect(self.path_entry.setToolTip)
        settings.addWidget(self.path_entry)
        self.browse = QPushButton('Browse folder...')
        self.browse.clicked.connect(self.choose_output)
        settings.addWidget(self.browse)
        duration_row = QHBoxLayout()
        duration_row.addWidget(label('Episode length', 'fieldLabel'))
        self.duration = QDoubleSpinBox()
        self.duration.setRange(.1, 86400)
        self.duration.setDecimals(1)
        self.duration.setValue(30)
        self.duration.setSuffix(' s')
        duration_row.addWidget(self.duration)
        settings.addLayout(duration_row)
        self.mock_check = QCheckBox('Mock mode (no hardware)')
        self.mock_check.setChecked(mock)
        self.mock_check.toggled.connect(lambda value: self.mode_badge.setText('MOCK MODE' if value else 'HARDWARE MODE'))
        settings.addWidget(self.mock_check)
        config_label = label(f'Config: {Path(self.config_path).name}', 'muted')
        config_label.setToolTip(self.config_path)
        settings.addWidget(config_label)
        side.addWidget(record_card)
        side.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(sidebar)
        scroll.setMinimumWidth(310)
        splitter.addWidget(scroll)

        workspace = QWidget()
        work = QVBoxLayout(workspace)
        work.setContentsMargins(10, 0, 0, 0)
        work.setSpacing(16)
        metrics = QHBoxLayout()
        self.metric_values = {}
        for key, title, initial in (('frames', 'SAVED FRAMES', '0'), ('elapsed', 'ELAPSED', '0.0 s'),
                                    ('fps', 'AVERAGE FPS', '--'), ('depth', 'VALID DEPTH', '--')):
            metric = QFrame()
            metric.setObjectName('card')
            box = QVBoxLayout(metric)
            box.setContentsMargins(16, 12, 16, 12)
            box.addWidget(label(title, 'eyebrow'))
            value = label(initial, 'metricValue')
            box.addWidget(value)
            self.metric_values[key] = value
            metrics.addWidget(metric)
        work.addLayout(metrics)
        camera_card, camera_layout = card('Camera feeds')
        camera_card.setMinimumHeight(340)
        camera_header = QHBoxLayout()
        self.feed_status = label('IDLE  /  no live feed', 'muted')
        camera_header.addWidget(self.feed_status)
        camera_header.addStretch()
        camera_header.addWidget(label('Preview ~10 Hz', 'muted'))
        camera_layout.addLayout(camera_header)
        self.tabs = QTabWidget()
        self.views = {}
        for spec in cfg['cameras']:
            tab = QWidget()
            tab_layout = QHBoxLayout(tab)
            tab_layout.setContentsMargins(0, 12, 0, 0)
            tab_layout.setSpacing(14)
            views = []
            for title, subtitle in (('RGB COLOR', 'Original color stream'),
                                    ('DEPTH', '0-2 m | Blue: near / Red: far / Black: invalid')):
                column = QVBoxLayout()
                column.addWidget(label(title, 'eyebrow'))
                view = ImageView()
                column.addWidget(view, 1)
                column.addWidget(label(subtitle, 'muted', True))
                tab_layout.addLayout(column, 1)
                views.append(view)
            self.tabs.addTab(tab, spec['name'])
            self.views[spec['name']] = views
        self.tabs.currentChanged.connect(self.update_depth_metric)
        camera_layout.addWidget(self.tabs, 1)
        work.addWidget(camera_card, 1)
        state_card, state_layout = card('Arm telemetry')
        state_card.setMinimumHeight(320)
        self.table = QTableWidget(7, 3)
        self.table.setHorizontalHeaderLabels(['Joint / position', 'Left follower', 'Right follower'])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(28)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.table.setShowGrid(False)
        self.table.setAlternatingRowColors(True)
        self.table.setFixedHeight(232)
        for i, motor in enumerate((*MOTORS, 'End-effector XYZ (m, FK)')):
            title = motor.replace('_', ' ').title() if i < 6 else motor
            if i < 6:
                title += ' (0-100)' if i == 5 else ' (deg)'
            self.table.setItem(i, 0, QTableWidgetItem(title))
            for col in (1, 2):
                item = QTableWidgetItem('--')
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(i, col, item)
        state_layout.addWidget(self.table)
        state_layout.addWidget(label('FK estimates use the configured frame of each arm.', 'muted', True))
        work.addWidget(state_card)
        workspace.setMinimumHeight(780)
        workspace_scroll = QScrollArea()
        workspace_scroll.setWidgetResizable(True)
        workspace_scroll.setFrameShape(QFrame.Shape.NoFrame)
        workspace_scroll.setWidget(workspace)
        splitter.addWidget(workspace_scroll)
        splitter.setSizes([350, 1000])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        footer = QFrame()
        footer.setObjectName('card')
        footer_layout = QVBoxLayout(footer)
        footer_layout.setContentsMargins(18, 14, 18, 14)
        footer_layout.setSpacing(10)
        action_row = QHBoxLayout()
        self.status = label('Connect devices individually, then record the active devices.', wrap=True)
        action_row.addWidget(self.status, 1)
        self.start_button = QPushButton('Start Recording')
        self.start_button.setObjectName('primary')
        self.start_button.clicked.connect(self.start)
        self.stop_button = QPushButton('Stop Recording')
        self.stop_button.setObjectName('stop')
        self.stop_button.clicked.connect(self.request_stop)
        self.stop_button.setEnabled(False)
        self.emergency_button = QPushButton('Emergency Stop')
        self.emergency_button.setObjectName('emergency')
        self.emergency_button.clicked.connect(self.emergency_stop)
        self.emergency_button.setEnabled(False)
        action_row.addWidget(self.start_button)
        action_row.addWidget(self.stop_button)
        action_row.addWidget(self.emergency_button)
        footer_layout.addLayout(action_row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(6)
        footer_layout.addWidget(self.progress)
        footer_layout.addWidget(label('Stop Recording keeps teleop active. Emergency Stop disconnects arm devices and stops commands.', 'muted', True))
        layout.addWidget(footer)
        self.session = HardwareSession(cfg)
        self.refresh_ports()
        self.update_controls()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)

    def is_running(self):
        return (self.pending or self.countdown_remaining > 0 or self.device_state['recording']
            or self.device_state.get('label_pending', False))

    def refresh_ports(self):
        if self.is_running():
            return
        ports = scan_ports()
        previous = self.detected_ports
        self.detected_ports = set(ports)
        for box in self.port_boxes.values():
            selected = box.currentText()
            box.clear()
            box.addItems(ports)
            box.setCurrentText(selected)
        message = f'{len(ports)} ports found. Assign each connected pair manually.'
        if previous is not None:
            added, removed = sorted(set(ports)-previous), sorted(previous-set(ports))
            if added:
                message += ' Added: ' + ', '.join(added)
            if removed:
                message += ' Removed: ' + ', '.join(removed)
        self.port_status.setText(message)

    def current_config(self):
        cfg = load_config(self.config_path)
        for (role, side), box in self.port_boxes.items():
            cfg[role]['ports'][side] = box.currentText().strip()
        return cfg

    def selected_config(self):
        return apply_ports(load_config(self.config_path), {key: box.currentText() for key, box in self.port_boxes.items()})

    def save_ports(self):
        if self.is_running():
            return
        try:
            write_json(self.config_path, self.selected_config())
            self.port_status.setText(f'Saved to {self.config_path}. Unused pairs may remain unset.')
        except Exception as error:
            self.port_status.setText(f'Save failed: {error}')

    def choose_output(self):
        folder = QFileDialog.getExistingDirectory(self, 'Select output folder', self.path_entry.text())
        if folder:
            self.path_entry.setText(folder)

    def send(self, operation, **kwargs):
        if self.pending or self.closing:
            return
        self.pending = True
        self.status.setText('Working: ' + operation + '...')
        self.update_controls()
        self.session.submit(operation, **kwargs)

    def toggle_device(self, kind, name):
        connected = name in self.device_state['sides' if kind == 'arm' else 'cameras']
        if connected:
            self.send('disconnect', kind=kind, name=name)
        else:
            try:
                cfg = self.current_config()
            except Exception as error:
                self.status.setText(f'Invalid config: {error}')
                return
            self.send('connect', kind=kind, name=name, config=cfg, mock=self.mock_check.isChecked())

    def toggle_teleop(self, side):
        self.send('teleop', side=side, enabled=side not in self.device_state['teleop'])

    def update_controls(self):
        locked = self.pending or self.device_state['recording'] or self.closing or not self.session.thread.is_alive()
        active = bool(self.device_state['sides'] or self.device_state['cameras'])
        for widget in (self.path_entry, self.browse, self.duration, self.scan_button, self.save_button):
            widget.setEnabled(not locked)
        self.mock_check.setEnabled(not locked and not active)
        self.start_button.setEnabled(not locked and active)
        self.stop_button.setEnabled(self.device_state['recording'] and not self.closing)
        self.emergency_button.setEnabled(active and not self.pending and not self.closing)
        for (role, side), box in self.port_boxes.items():
            box.setEnabled(not locked and side not in self.device_state['sides'])
        for side, button in self.arm_buttons.items():
            connected = side in self.device_state['sides']
            moving = side in self.device_state['teleop']
            button.setText('Disconnect' if connected else 'Connect')
            button.setEnabled(not locked)
            self.teleop_buttons[side].setText('Stop Teleop' if moving else 'Start Teleop')
            self.teleop_buttons[side].setEnabled(not locked and connected)
            self.arm_status[side].setText('Teleop active' if moving else 'Connected / read-only' if connected else 'Disconnected')
        for name, button in self.camera_buttons.items():
            button.setText('Disconnect' if name in self.device_state['cameras'] else 'Connect')
            button.setEnabled(not locked)

    def choose_sample_label(self, directory):
        choice = QMessageBox.question(
            self, 'Classify episode',
            f'Was this episode a successful demonstration?\n\n{directory}\n\nYes = positive, No = negative',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes,
        )
        self.send('label', sample_label='positive' if choice == QMessageBox.StandardButton.Yes else 'negative')

    def start(self):
        if self.is_running():
            return
        if not self.path_entry.text().strip():
            self.status.setText('Set an output folder')
            return
        self.countdown_remaining = 3
        self.status.setText('Recording starts in 3...')
        self.update_controls()
        self.countdown_timer.start(1000)

    def countdown_tick(self):
        self.countdown_remaining -= 1
        if self.countdown_remaining > 0:
            self.status.setText(f'Recording starts in {self.countdown_remaining}...')
            return
        self.countdown_timer.stop()
        self.session.stop_recording.clear()
        self.progress.setRange(0, int(self.duration.value()*self.session.config['fps']))
        self.progress.setValue(0)
        self.send('record', output=str(Path(self.path_entry.text()).expanduser().resolve()), seconds=self.duration.value())

    def request_stop(self):
        self.session.stop_recording.set()
        self.stop_button.setEnabled(False)
        self.status.setText('Stopping recording; teleoperation remains active...')

    def emergency_stop(self):
        self.countdown_timer.stop()
        self.countdown_remaining = 0
        self.send('emergency_stop')

    def closeEvent(self, event):
        if self.session.thread.is_alive():
            self.closing = True
            self.session.shutdown.set()
            self.status.setText('Closing: saving episode and disconnecting devices...')
            self.update_controls()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()

    def clear_inactive(self):
        for name, views in self.views.items():
            if name not in self.device_state['cameras']:
                for view in views:
                    view.clear()
        for i, side in enumerate(('left', 'right')):
            if side not in self.device_state['sides']:
                for row in range(7):
                    self.table.item(row, i+1).setText('--')

    def render_frame(self, values, meta, received):
        self.last_frame_at = received
        self.progress.setValue(meta['frames'])
        self.metric_values['frames'].setText(str(meta['frames']))
        self.metric_values['elapsed'].setText(f'{meta["elapsed"]:.1f} s')
        self.metric_values['fps'].setText(f'{meta["fps"]:.1f}')
        self.depth_stats = {}
        for name in meta['cameras']:
            views = self.views[name]
            raw = values[f'{name}__depth']
            valid = raw > 0
            t = np.clip(raw * meta['camera_metadata'][name]['depth_scale_m'] / 2, 0, 1)
            color = np.stack((255*t, 255*(1-abs(2*t-1)), 255*(1-t)), axis=-1).astype(np.uint8)
            color[~valid] = 0
            views[0].set_frame(values[f'{name}__rgb'])
            views[1].set_frame(color)
            self.depth_stats[name] = valid.mean()
        self.update_depth_metric()
        for offset, side in enumerate(meta['sides']):
            col = 1 if side == 'left' else 2
            for i in range(6):
                self.table.item(i, col).setText(f'{values["joints"][offset*6+i]:.2f}')
            xyz = values['ee_transform'][offset, :3, 3]
            self.table.item(6, col).setText(', '.join(f'{x:.4f}' for x in xyz) if np.isfinite(xyz).all() else 'FK not configured')
        mode = 'MOCK' if meta['mock'] else 'HARDWARE'
        self.feed_status.setText(f'{mode} / ' + ('RECORDING' if meta['recording'] else 'LIVE PREVIEW'))

    def update_depth_metric(self):
        name = self.tabs.tabText(self.tabs.currentIndex())
        ratio = self.depth_stats.get(name) if isinstance(self.depth_stats, dict) else None
        self.metric_values['depth'].setText(f'{ratio:.1%}' if ratio is not None else '--')

    def poll(self):
        try:
            frame = self.session.frames.get_nowait()
        except queue.Empty:
            frame = None
        if frame is not None:
            try:
                self.render_frame(*frame)
            except Exception as error:
                self.session.shutdown.set()
                self.status.setText(f'Preview failed; disconnecting devices: {error}')
        while True:
            try:
                event, value = self.session.events.get_nowait()
            except queue.Empty:
                break
            if event == 'status':
                self.status.setText(value)
            elif event == 'devices':
                self.device_state = value
                self.clear_inactive()
            elif event == 'command_done':
                self.pending = False
            elif event == 'label_required':
                self.choose_sample_label(value)
        self.update_controls()
        if not self.device_state['cameras'] and not self.device_state['sides']:
            self.feed_status.setText('IDLE / no connected devices')
            self.metric_values['depth'].setText('--')
        elif self.last_frame_at and time.monotonic()-self.last_frame_at > 1:
            self.feed_status.setText('WAITING / no new frames for over 1 s')
        if self.closing and not self.session.thread.is_alive():
            self.close()


def main():
    parser = argparse.ArgumentParser(description='PyQt6 RGB-D recording studio')
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    parser.add_argument('--output', type=Path, default=ROOT / 'data/recordings')
    parser.add_argument('--mock', action='store_true')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    window = RecorderUI(args.config, args.output, args.mock)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
