"""Read coherent RGB and aligned raw Z16 depth from RealSense framesets."""
import time
import numpy as np


class RealSense:
    def __init__(self, config):
        self.config = config
        self.pipeline = None

    def connect(self):
        import pyrealsense2 as rs
        self.rs = rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.config['serial'])
        w, h, fps = (self.config[k] for k in ('width', 'height', 'fps'))
        config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        profile = pipeline.start(config)
        self.pipeline = pipeline
        self.align = rs.align(rs.stream.color)
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.metadata = {
            'serial': self.config['serial'], 'depth_scale_m': profile.get_device().first_depth_sensor().get_depth_scale(),
            'depth_aligned_to': 'color', 'depth_invalid_value': 0,
            'color_intrinsics': {k: getattr(intrinsics, k) for k in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy', 'coeffs')},
            'distortion_model': str(intrinsics.model),
            'camera_to_world': None,
        }
        for _ in range(fps):
            pipeline.wait_for_frames(timeout_ms=5000)

    def read(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=2000)
        received = time.monotonic()
        aligned = self.align.process(frames)
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        if not color or not depth:
            raise RuntimeError('Missing RGB/depth in RealSense frameset')
        return {
            'rgb': np.asanyarray(color.get_data()).copy(),
            'depth': np.asanyarray(depth.get_data()).copy(),
            'camera_host_time': np.float64(received),
            'color_device_ms': np.float64(color.get_timestamp()),
            'depth_device_ms': np.float64(depth.get_timestamp()),
            'color_frame_number': np.int64(color.get_frame_number()),
            'depth_frame_number': np.int64(depth.get_frame_number()),
            'color_clock_domain': np.str_(str(color.get_frame_timestamp_domain())),
            'depth_clock_domain': np.str_(str(depth.get_frame_timestamp_domain())),
        }

    def close(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


def main():
    import argparse
    import json
    from .common import ROOT
    parser = argparse.ArgumentParser(description='Check RGB-D streams without connecting robot arms')
    parser.add_argument('--config', default=str(ROOT / 'configs/rgbd.json'))
    args = parser.parse_args()
    config = json.loads(open(args.config).read())
    for spec in config['cameras']:
        camera = RealSense(spec)
        try:
            camera.connect()
            frame = camera.read()
            depth = frame['depth']
            print(f'{spec["name"]}: RGB {frame["rgb"].shape} {frame["rgb"].dtype}; depth {depth.shape} {depth.dtype}; valid={(depth > 0).mean():.1%}; scale={camera.metadata["depth_scale_m"]} m/unit', flush=True)
        finally:
            camera.close()


if __name__ == '__main__':
    main()
