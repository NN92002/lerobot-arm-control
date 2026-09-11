import json
import unittest
from control import ROOT, build_command


class CommandTests(unittest.TestCase):
    def config(self, mode='dual'):
        return json.loads((ROOT / 'configs' / f'{mode}.json').read_text())

    def test_dual(self):
        command, ports = build_command(self.config(), 'teleoperate')
        self.assertEqual(len(ports), 4)
        self.assertIn('--robot.type=bi_so_follower', command)
        self.assertIn('--teleop.type=bi_so_leader', command)

    def test_project_environment_python(self):
        command, _ = build_command(self.config(), 'teleoperate')
        self.assertEqual(command[0], str(ROOT / '.conda/bin/python'))
        self.assertIsNone(self.config()['lerobot_path'])

    def test_selected_pair_uses_recording_calibration_identity(self):
        command, ports = build_command(self.config(), 'calibrate-robot', side='right')
        self.assertEqual(len(ports), 1)
        self.assertIn('--robot.type=so101_follower', command)
        self.assertIn('--robot.id=dual_robot_right', command)
        self.assertIn(f'--robot.calibration_dir={ROOT / "calibration/dual/robot"}', command)
        self.assertFalse(any('left' in value for value in ports))

    def test_single(self):
        command, ports = build_command(self.config('single'), 'teleoperate')
        self.assertEqual(len(ports), 2)
        self.assertIn('--robot.type=so101_follower', command)

    def test_calibration_only_selected_role(self):
        command, ports = build_command(self.config(), 'calibrate-robot')
        self.assertEqual(len(ports), 2)
        self.assertFalse(any(c.startswith('--teleop.') for c in command))

    def test_duplicate_ports_rejected(self):
        config = self.config()
        config['robot']['ports']['right'] = config['robot']['ports']['left']
        with self.assertRaises(ValueError):
            build_command(config, 'teleoperate')

    def test_invalid_limits(self):
        for value in (0, -1, float('nan'), True):
            config = self.config()
            config['max_relative_target'] = value
            with self.assertRaises(ValueError):
                build_command(config, 'teleoperate')
