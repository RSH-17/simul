"""Import the teammate's Mixed v1.5.2.57 release package as an isolated team-sim workspace.

The package is a complete workspace (its own 50-stage v1.5.2 lineage), so the
`parent` argument that setup.py passes for overlay candidates is not used.
Prebuilt binaries in the archive are removed; setup.py rebuilds them from source.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile

TAG = 'policy-v1.5.2.57'
REPOSITORY = 'peter010905/simul'
ASSET = 'osp-policy-v1.5.2.57.tar.gz'
ARCHIVE_SHA256 = 'f959bde5d38513a48c4c6f6ab144558d2dab264887fda38ced886dc28cbe0e88'
ROOT = 'osp-policy-v1.5.2.57'
WORKSPACE = 'v15257_short_lap_lidar5_640_1126_ws'
PREBUILT = ('src/osp_mission/osp_mission/_raycast.so', 'src/osp_mission/tools/export_slam_graph_poses')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe_extract(archive, target):
    with tarfile.open(archive) as tar:
        for item in tar.getmembers():
            name = PurePosixPath(item.name)
            if name.is_absolute() or '..' in name.parts or item.issym() or item.islnk() \
                    or not (item.isfile() or item.isdir()) or name.parts[0] != ROOT:
                raise ValueError('Unsafe archive member: ' + item.name)
        tar.extractall(target, filter='data')


def verify_sums(package):
    for line in (package / 'SHA256SUMS').read_text().splitlines():
        value, name = line.split(maxsplit=1)
        if sha(package / name) != value:
            raise ValueError('Package checksum mismatch: ' + name)


def install(destination, archive=None):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        temporary = Path(temporary)
        if archive is None:
            subprocess.run(['gh', 'release', 'download', TAG, '-R', REPOSITORY, '-p', ASSET,
                            '-D', str(temporary / 'download')], check=True)
            archive = temporary / 'download' / ASSET
        if sha(archive) != ARCHIVE_SHA256:
            raise ValueError('Release archive SHA-256 mismatch')
        safe_extract(archive, temporary / 'extract')
        package = temporary / 'extract' / ROOT
        verify_sums(package)
        source = package / WORKSPACE
        stage = temporary / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(source / name, stage / name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for file in source.glob('*_manifest.json'):
            shutil.copyfile(file, stage / file.name)
        for name in PREBUILT:
            (stage / name).unlink(missing_ok=True)
        short_lap = json.loads((stage / 'short_lap_manifest.json').read_text())
        if short_lap['version'] != '1.5.2.57-short-lap':
            raise ValueError('Unexpected package version: ' + short_lap['version'])
        (stage / 'v15257_import_manifest.json').write_text(json.dumps(dict(
            version='1.5.2.57-short-lap (imported, unchanged source)',
            release=f'https://github.com/{REPOSITORY}/releases/tag/{TAG}',
            archive_sha256=ARCHIVE_SHA256,
            removed_prebuilt=list(PREBUILT),
            policy_source_changed=False,
            physical_drive=False), indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent', type=Path, help='ignored; kept for the setup.py candidate interface')
    p.add_argument('destination', type=Path)
    p.add_argument('--archive', type=Path, help='use an already downloaded release archive')
    a = p.parse_args()
    install(a.destination, a.archive)
