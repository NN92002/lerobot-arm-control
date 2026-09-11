import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from rgbd.common import ROOT
from rgbd.record import record_episode


class PortTests(unittest.TestCase):
    def test_alias_duplicate_rejected(self):
        from rgbd.gui import apply_ports, ARM_PORTS
        cfg = json.loads((ROOT / 'configs/rgbd.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            device = Path(tmp) / 'ttyACM0'
            device.touch()
            alias = Path(tmp) / 'usb-arm'
            alias.symlink_to(device)
            ports = {(role, side): f'/dev/ttyACM{i}' for i, (role, side, _) in enumerate(ARM_PORTS)}
            ports['teleop', 'left'] = str(device)
            ports['robot', 'left'] = str(alias)
            with self.assertRaisesRegex(ValueError, 'same port'):
                apply_ports(cfg, ports)


class StopTests(unittest.TestCase):
    def test_stop_preserves_saved_frame_and_marks_incomplete(self):
        cfg = json.loads((ROOT / 'configs/rgbd.json').read_text())
        cfg['cameras'][0].update(width=16, height=16)
        stop = threading.Event()
        with tempfile.TemporaryDirectory() as tmp:
            episode = Path(tmp) / 'episode_000000'
            record_episode(episode, cfg, 10, {'front': SimpleNamespace(metadata={'depth_scale_m': .001})},
                           {}, {}, mock=True, on_frame=lambda *_: stop.set(), stop_event=stop)
            meta = json.loads((episode / 'episode.json').read_text())
            self.assertFalse(meta['complete'])
            self.assertTrue(meta['stopped_by_user'])
            self.assertEqual(meta['frames'], 1)
            self.assertEqual(len(list(episode.glob('*.npz'))), 1)


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        cls.app.setStyle('Fusion')

    def make_ui(self, config, output):
        from rgbd.gui import RecorderUI
        ui = RecorderUI(config, output, mock=True)
        ui.show()
        self.app.processEvents()
        self.addCleanup(self.cleanup_ui, ui)
        return ui

    def cleanup_ui(self, ui):
        ui.session.shutdown.set()
        ui.session.thread.join(timeout=5)
        ui.poll()
        ui.close()
        self.app.processEvents()

    def wait_for(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(.01)
        self.fail('Timed out waiting for Qt / session')

    def connect(self, ui, kind, name):
        ui.toggle_device(kind, name)
        self.wait_for(lambda: not ui.pending)
        self.assertIn(name, ui.device_state['sides' if kind == 'arm' else 'cameras'], ui.status.text())

    def test_port_scan_save_and_lock(self):
        from rgbd.gui import ARM_PORTS
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'rgbd.json'
            original = json.loads((ROOT / 'configs/rgbd.json').read_text())
            config.write_text(json.dumps(original))
            with patch('rgbd.gui.scan_ports', return_value=['/dev/ttyACM0']):
                ui = self.make_ui(config, Path(tmp))
            with patch('rgbd.gui.scan_ports', return_value=['/dev/ttyACM0', '/dev/ttyACM1']):
                ui.refresh_ports()
            self.assertIn('Added: /dev/ttyACM1', ui.port_status.text())
            for i, (role, side, _) in enumerate(ARM_PORTS):
                ui.port_boxes[role, side].setCurrentText(f'/dev/ttyACM{i}')
            ui.save_button.click()
            saved = json.loads(config.read_text())
            self.assertEqual(saved['robot']['ports']['right'], '/dev/ttyACM3')
            self.assertEqual(saved['kinematics'], original['kinematics'])
            ui.port_boxes['robot', 'left'].setCurrentText('/dev/ttyACM0')
            ui.save_button.click()
            self.assertIn('same port', ui.port_status.text())
            self.assertEqual(json.loads(config.read_text()), saved)
            ui.port_boxes['robot', 'left'].setCurrentText('/dev/ttyACM1')
            self.connect(ui, 'arm', 'left')
            self.assertFalse(ui.port_boxes['robot', 'left'].isEnabled())
            self.assertTrue(ui.port_boxes['robot', 'right'].isEnabled())
            self.assertFalse(ui.mock_check.isEnabled())

    def test_camera_preview_then_right_only_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            ui = self.make_ui(ROOT / 'configs/rgbd.json', Path(tmp))
            self.assertFalse(ui.start_button.isEnabled())
            self.connect(ui, 'camera', 'front')
            self.wait_for(lambda: not ui.views['front'][0].image.isNull())
            self.assertFalse(list(Path(tmp).glob('episode_*')))
            self.connect(ui, 'arm', 'right')
            ui.toggle_teleop('right')
            self.wait_for(lambda: not ui.pending)
            ui.duration.setValue(.3)
            ui.start_button.click()
            self.wait_for(lambda: ui.device_state['recording'])
            self.assertFalse(ui.arm_buttons['left'].isEnabled())
            self.wait_for(lambda: not ui.is_running())
            self.assertIn('Recording complete', ui.status.text())
            self.assertEqual(ui.device_state['sides'], ['right'])
            self.assertEqual(ui.device_state['teleop'], [])
            meta = json.loads((Path(tmp) / 'episode_000000/episode.json').read_text())
            self.assertTrue(meta['complete'])
            self.assertEqual(meta['active_sides'], ['right'])
            self.assertEqual(len(meta['joint_names']), 6)
            self.assertEqual(ui.table.item(0, 1).text(), '--')
            self.assertNotEqual(ui.table.item(0, 2).text(), '--')
            self.assertEqual(ui.progress.value(), 9)
            # Next episode uses the newly selected hardware, not the previous layout.
            ui.toggle_device('arm', 'right')
            self.wait_for(lambda: not ui.pending)
            ui.start_button.click()
            self.wait_for(lambda: ui.device_state['recording'])
            self.wait_for(lambda: not ui.is_running())
            meta = json.loads((Path(tmp) / 'episode_000001/episode.json').read_text())
            self.assertEqual(meta['active_sides'], [])

    def test_close_during_recording_preserves_partial_episode(self):
        with tempfile.TemporaryDirectory() as tmp:
            ui = self.make_ui(ROOT / 'configs/rgbd.json', Path(tmp))
            self.connect(ui, 'arm', 'left')
            ui.duration.setValue(10)
            ui.start_button.click()
            self.wait_for(lambda: ui.progress.value() >= 2)
            ui.close()
            self.assertTrue(ui.closing)
            self.wait_for(lambda: not ui.isVisible())
            self.assertFalse(ui.session.thread.is_alive())
            meta = json.loads((Path(tmp) / 'episode_000000/episode.json').read_text())
            self.assertFalse(meta['complete'])
            self.assertTrue(meta['stopped_by_user'])

    def test_writer_failure_unlocks_controls_and_disconnects(self):
        with tempfile.TemporaryDirectory() as tmp:
            ui = self.make_ui(ROOT / 'configs/rgbd.json', Path(tmp))
            self.connect(ui, 'camera', 'front')
            with patch('rgbd.session.EpisodeWriter.append', side_effect=OSError('Disk unavailable')):
                ui.start_button.click()
                self.wait_for(lambda: not ui.pending and not ui.device_state['cameras'])
            self.assertIn('Disk unavailable', ui.status.text())
            self.assertTrue(ui.camera_buttons['front'].isEnabled())
            self.assertFalse(ui.stop_button.isEnabled())
