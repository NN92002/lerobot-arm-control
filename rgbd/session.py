"""Single-owner device thread: independent connections, preview, and recording."""
from contextlib import ExitStack
import copy
import json
from pathlib import Path
import queue
import threading
import time
from .camera import RealSense
from .record import (select_config, make_arms, disconnect, validate_config, build_solvers,
                     mock_cameras, sample_frame, EpisodeWriter)
from .common import write_json
from .transfer import next_remote_episode_index, upload_episode


class HardwareSession:
    def __init__(self, config):
        self.config = copy.deepcopy(config)
        self.commands = queue.Queue()
        self.events = queue.Queue()
        self.frames = queue.Queue(maxsize=1)
        self.shutdown = threading.Event()
        self.stop_recording = threading.Event()
        self.cameras, self.arms, self.solvers, self.resources = {}, {}, {}, {}
        self.sides, self.teleop = set(), set()
        self.mock = False
        self.writer = None
        self.pending_episode = None
        self.target_frames = 0
        self.last_recording = {'frames': 0, 'elapsed': 0., 'fps': 0.}
        self.thread = threading.Thread(target=self.loop, daemon=False)
        self.thread.start()

    def submit(self, operation, **kwargs):
        self.commands.put((operation, kwargs))

    def snapshot(self):
        return dict(sides=sorted(self.sides), cameras=list(self.cameras), teleop=sorted(self.teleop),
                    recording=self.writer is not None, label_pending=self.pending_episode is not None,
                    mock=self.mock)

    def emit_state(self):
        self.events.put(('devices', self.snapshot()))

    def selected(self):
        return select_config(self.config, self.sides, self.cameras)

    def connect(self, kind, name, config, mock):
        if self.writer:
            raise ValueError('Stop recording before changing devices')
        if self.resources and self.mock != mock:
            raise ValueError('Disconnect all devices before changing mock mode')
        key = (kind, name)
        if key in self.resources:
            return
        cfg = copy.deepcopy(config)
        if kind == 'arm':
            validate = select_config(cfg, self.sides | {name}, [])
        else:
            validate = select_config(cfg, [], [*self.cameras, name])
        if not mock:
            validate_config(validate)
        stack = ExitStack()
        try:
            if kind == 'camera':
                one = select_config(cfg, [], [name])
                if mock:
                    camera = mock_cameras(one)[name]
                else:
                    camera = RealSense(one['cameras'][0])
                    stack.callback(camera.close)
                    camera.connect()
                self.cameras[name] = camera
            else:
                one = select_config(cfg, [name], [])
                arms = {} if mock else make_arms(one)
                for arm in arms.values():
                    stack.callback(disconnect, arm)
                    arm.connect(calibrate=False)
                    if not arm.is_calibrated:
                        raise ValueError('Calibrate this arm pair first')
                # Connections and joint preview work before an URDF is configured.
                try:
                    solvers = {} if mock else build_solvers(one)
                except (ValueError, OSError, KeyError):
                    solvers = {}
                self.arms.update(arms)
                self.solvers.update(solvers)
                self.sides.add(name)
            self.config = cfg
            self.mock = mock
            self.resources[key] = stack.pop_all()
        finally:
            stack.close()
        self.events.put(('status', f'{name}: connected. Arm teleoperation starts only with Start Teleop.'))

    def disconnect_one(self, kind, name):
        if self.writer:
            raise ValueError('Stop recording before disconnecting devices')
        stack = self.resources.pop((kind, name), None)
        try:
            if stack:
                stack.close()
        finally:
            if kind == 'camera':
                self.cameras.pop(name, None)
            else:
                self.teleop.discard(name)
                self.sides.discard(name)
                self.solvers.pop(name, None)
                for role in ('robot', 'teleop'):
                    self.arms.pop((role, name), None)

    def finish_recording(self, reason=None):
        if self.writer is None:
            return
        writer, self.writer = self.writer, None
        # Recording completion/stop also stops issuing teleoperation targets.
        self.teleop.clear()
        complete = reason is None and writer.meta['frames'] == self.target_frames
        writer.close(complete, reason)
        message = f'{"Recording complete; choose sample label" if complete else "Incomplete episode kept"}: {writer.directory}. Teleop stopped; devices remain connected.'
        if complete and not self.mock:
            self.pending_episode = writer.directory
            self.events.put(('label_required', str(writer.directory)))
        self.events.put(('status', message))
        self.emit_state()

    def handle(self, operation, args):
        if operation == 'connect':
            self.connect(**args)
        elif operation == 'disconnect':
            self.disconnect_one(**args)
        elif operation == 'teleop':
            if self.writer:
                raise ValueError('Stop recording before changing teleoperation')
            side, enabled = args['side'], args['enabled']
            if side not in self.sides:
                raise ValueError('Connect the arm pair first')
            if enabled:
                self.teleop.add(side)
            else:
                self.teleop.discard(side)
        elif operation == 'record':
            if self.writer:
                raise ValueError('Already recording')
            if self.pending_episode:
                raise ValueError('Label the completed episode before recording another')
            if not self.resources:
                raise ValueError('Connect at least one arm pair or camera first')
            cfg = self.selected()
            if not self.mock:
                self.solvers = build_solvers(cfg)
            frames = int(args['seconds'] * cfg['fps'])
            if frames < 2:
                raise ValueError('An episode needs at least two frames')
            output = Path(args['output'])
            output.mkdir(parents=True, exist_ok=True)
            index = 0
            while (output / f'episode_{index:06d}').exists():
                index += 1
            server = self.config.get('server', {})
            if not self.mock and server.get('auto_upload', False):
                index = next_remote_episode_index(server, output.name, output)
            self.writer = EpisodeWriter(output / f'episode_{index:06d}', cfg, self.cameras, self.mock, sorted(self.teleop))
            self.target_frames = frames
            self.record_started = time.monotonic()
            self.last_recording = {'frames': 0, 'elapsed': 0., 'fps': 0.}
            self.events.put(('status', f'Recording: {self.writer.directory}'))
        elif operation == 'label':
            if self.pending_episode is None:
                raise ValueError('No completed episode is waiting for a label')
            sample_label = args['sample_label']
            if sample_label not in ('positive', 'negative'):
                raise ValueError('sample_label must be positive or negative')
            directory = self.pending_episode
            meta_path = directory / 'episode.json'
            meta = json.loads(meta_path.read_text())
            meta['sample_label'] = sample_label
            write_json(meta_path, meta)
            server = self.config.get('server', {})
            if server.get('auto_upload', False):
                self.events.put(('status', f'{sample_label.title()} sample: {directory}. Uploading...'))
                try:
                    remote_directory = upload_episode(directory, server)
                    import shutil
                    shutil.rmtree(directory)
                    message = f'{sample_label.title()} sample uploaded and removed locally: {remote_directory}'
                except Exception as error:
                    message = f'{sample_label.title()} sample upload failed; local episode kept at {directory}: {error}'
            else:
                message = f'{sample_label.title()} sample saved locally: {directory}'
            self.pending_episode = None
            self.events.put(('status', message))
        else:
            raise ValueError(f'Unknown operation: {operation}')

    def close_devices(self):
        errors = []
        for kind, name in list(self.resources):
            try:
                self.disconnect_one(kind, name)
            except Exception as error:
                errors.append(str(error))
        return errors

    def loop(self):
        index = 0
        try:
            while not self.shutdown.is_set():
                started = time.monotonic()
                if self.stop_recording.is_set():
                    try:
                        self.finish_recording('user')
                    finally:
                        self.stop_recording.clear()
                try:
                    operation, args = self.commands.get(timeout=0 if self.resources else .05)
                except queue.Empty:
                    operation = None
                if operation:
                    try:
                        self.handle(operation, args)
                    except Exception as error:
                        self.events.put(('status', f'Operation failed: {error}'))
                    finally:
                        self.emit_state()
                        self.events.put(('command_done', None))
                if self.shutdown.is_set():
                    break
                if not self.resources:
                    continue
                try:
                    cfg = self.selected()
                    values = sample_frame(cfg, self.cameras, self.arms, self.solvers, self.mock, index, self.teleop, self.shutdown)
                    index += 1
                    # GUI uses wall-clock capture times even for a mock live session.
                    if self.mock:
                        now = time.monotonic()
                        values['host_time'] = now
                        if 'action_host_time' in values:
                            values['action_host_time'] = now
                        for name in self.cameras:
                            values[f'{name}__camera_host_time'] = now
                    if self.writer:
                        self.writer.append(values)
                        elapsed = max(time.monotonic()-self.record_started, .001)
                        self.last_recording = dict(frames=self.writer.meta['frames'], elapsed=elapsed,
                                                   fps=self.writer.meta['frames']/elapsed)
                        if self.writer.meta['frames'] >= self.target_frames:
                            self.finish_recording()
                    preview = dict(self.snapshot(), camera_metadata={n: c.metadata for n, c in self.cameras.items()},
                                   **self.last_recording)
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                    self.frames.put_nowait((values, preview, time.monotonic()))
                except Exception as error:
                    if self.shutdown.is_set() and isinstance(error, InterruptedError):
                        break
                    self.teleop.clear()
                    try:
                        self.finish_recording('error')
                    finally:
                        cleanup_errors = self.close_devices()
                    self.events.put(('status', f'Device/read/write failure: {error}. Devices disconnected. {"; ".join(cleanup_errors)}'))
                    self.emit_state()
                self.shutdown.wait(max(0, started+1/self.config['fps']-time.monotonic()))
        except Exception as error:
            self.events.put(('status', f'Session failed: {error}'))
        finally:
            try:
                self.finish_recording('user')
            except Exception as error:
                self.events.put(('status', f'Could not finalize episode: {error}'))
            self.close_devices()
            self.emit_state()
            self.events.put(('closed', None))
