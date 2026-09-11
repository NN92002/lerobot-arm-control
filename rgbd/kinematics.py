"""Small URDF forward-kinematics reader; no mesh or physics dependencies."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(norm) or norm == 0:
        raise ValueError('URDF joint axis must be finite and nonzero')
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def xyz(text):
    values = np.asarray([float(x) for x in text.split()])
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError('Invalid URDF xyz/rpy/axis')
    return values


class ForwardKinematics:
    def __init__(self, config):
        self.config = config
        path = Path(config['urdf'])
        tree = ET.parse(path).getroot()
        joints = {}
        for node in tree.findall('joint'):
            child = node.find('child').get('link')
            if child in joints:
                raise ValueError('URDF has multiple parents')
            joints[child] = node
        current = config['tip_link']
        self.chain = []
        visited = set()
        while current != config['base_link']:
            if current in visited or current not in joints:
                raise ValueError(f'Cannot find URDF chain to {current}')
            visited.add(current)
            node = joints[current]
            kind = node.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic') or node.find('mimic') is not None:
                raise ValueError(f'Unsupported URDF joint: {node.get("name")} ({kind}/mimic)')
            if kind != 'fixed' and node.get('name') not in config['joint_map']:
                raise ValueError(f'Missing joint_map for {node.get("name")}')
            self.chain.append(node)
            current = node.find('parent').get('link')
        self.chain.reverse()
        self.base = np.asarray(config.get('world_from_base', np.eye(4)), dtype=float)
        if (self.base.shape != (4, 4) or not np.isfinite(self.base).all()
                or not np.allclose(self.base[3], [0, 0, 0, 1])
                or not np.allclose(self.base[:3, :3].T @ self.base[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(self.base[:3, :3]), 1)):
            raise ValueError('world_from_base must be a rigid 4x4 transform')

    def compute(self, motor_degrees):
        transform = self.base.copy()
        for node in self.chain:
            origin = node.find('origin')
            local = np.eye(4)
            if origin is not None:
                local[:3, 3] = xyz(origin.get('xyz', '0 0 0'))
                roll, pitch, yaw = xyz(origin.get('rpy', '0 0 0'))
                local[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
            motion = np.eye(4)
            kind = node.get('type')
            if kind != 'fixed':
                mapping = self.config['joint_map'][node.get('name')]
                # Revolute: degrees -> radians; prismatic mappings must explicitly supply metres/unit.
                scale = mapping.get('scale', np.pi / 180 if kind != 'prismatic' else None)
                if scale is None:
                    raise ValueError('Prismatic joint_map requires an explicit scale in metres/unit')
                value = motor_degrees[mapping['motor']] * scale + mapping.get('offset', 0)
                if not np.isfinite(value):
                    raise ValueError('Nonfinite joint position')
                axis_node = node.find('axis')
                axis = xyz(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0')
                if kind == 'prismatic':
                    motion[:3, 3] = axis / np.linalg.norm(axis) * value
                else:
                    motion[:3, :3] = rotation(axis, value)
            transform = transform @ local @ motion
        return transform.astype(np.float32)
