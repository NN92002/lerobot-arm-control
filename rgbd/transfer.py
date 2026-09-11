"""Upload completed episodes and verify them before removing local copies."""
import shlex
import subprocess
from pathlib import Path


def _local_manifest(directory):
    return {str(path.relative_to(directory)): path.stat().st_size
            for path in directory.rglob('*') if path.is_file()}


def _remote_manifest(ssh_target, directory):
    command = f"cd {shlex.quote(directory)} && find . -type f -printf '%P\\t%s\\n' | sort"
    result = subprocess.run(['ssh', ssh_target, command], check=True, capture_output=True, text=True)
    manifest = {}
    for line in result.stdout.splitlines():
        relative, size = line.split('\t', 1)
        manifest[relative] = int(size)
    return manifest


def next_remote_episode_index(server, dataset_name, local_directory=None):
    ssh_target = server.get('ssh')
    dataset_root = server.get('dataset_path')
    if not ssh_target or not dataset_root:
        raise ValueError('server.ssh and server.dataset_path are required for upload')
    remote_dataset = f'{dataset_root.rstrip("/")}/{dataset_name}'
    command = (f"mkdir -p {shlex.quote(remote_dataset)} && "
               f"find {shlex.quote(remote_dataset)} -mindepth 1 -maxdepth 1 -type d "
               "-name 'episode_*' -printf '%f\\n' | sort")
    result = subprocess.run(['ssh', ssh_target, command], check=True, capture_output=True, text=True)
    indices = set()
    for name in result.stdout.splitlines():
        try:
            indices.add(int(name.removeprefix('episode_')))
        except ValueError:
            continue
    if local_directory is not None:
        for path in Path(local_directory).glob('episode_*'):
            try:
                indices.add(int(path.name.removeprefix('episode_')))
            except ValueError:
                continue
    index = 0
    while index in indices:
        index += 1
    return index


def upload_episode(directory, server):
    directory = Path(directory).resolve()
    if not directory.is_dir():
        raise ValueError(f'Episode directory not found: {directory}')
    ssh_target = server.get('ssh')
    dataset_root = server.get('dataset_path')
    if not ssh_target or not dataset_root:
        raise ValueError('server.ssh and server.dataset_path are required for upload')
    remote_directory = f'{dataset_root.rstrip("/")}/{directory.parent.name}/{directory.name}'
    remote_parent = str(Path(remote_directory).parent)
    subprocess.run(['ssh', ssh_target, f'mkdir -p {shlex.quote(remote_parent)}'], check=True)
    subprocess.run(['rsync', '-a', '--checksum', '--delete', '--partial',
                    f'{directory}/', f'{ssh_target}:{remote_directory}/'], check=True)
    local_manifest = _local_manifest(directory)
    remote_manifest = _remote_manifest(ssh_target, remote_directory)
    if local_manifest != remote_manifest:
        raise RuntimeError('Remote episode verification failed; local files were kept')
    return remote_directory