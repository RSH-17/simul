"""Materialize the Gazebo-tested package in a fresh, independent ROS workspace."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory


def prepare(workspace):
    candidate = Path(__file__).resolve().parent
    base = candidate.parents[1] / 'src/osp_mission'
    if workspace.exists():
        raise FileExistsError(workspace)
    with TemporaryDirectory(dir=workspace.parent) as tmp:
        stage = Path(tmp) / 'workspace'
        package = stage / 'src/osp_mission'
        for name, digest in json.loads((candidate / 'manifest.json').read_text()).items():
            source = candidate / 'overlay' / name
            if not source.exists():
                source = base / name
            if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
                raise ValueError(f'{name} differs from the tested package')
            target = package / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        stage.rename(workspace)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    prepare(parser.parse_args().workspace.resolve())
