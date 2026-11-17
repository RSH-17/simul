"""Build a checked face_intent_v2 adapter in a fresh workspace; never drive."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

from runtime_changes import apply


def build(workspace, run_tests):
    candidate = Path(__file__).resolve().parent
    release = candidate.parents[1]/'policy_releases/2026-11-13_face_intent_v2'
    for relative, expected in json.loads((release/'release.json').read_text())['files'].items():
        if hashlib.sha256((release/relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Frozen release changed: {relative}')
    manifest = json.loads((candidate/'candidate.json').read_text())
    for relative, expected in manifest['overlay_files'].items():
        if hashlib.sha256((candidate/'overlay'/relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Candidate overlay changed: {relative}')
    if hashlib.sha256((candidate/'runtime_changes.py').read_bytes()).hexdigest() != manifest['transform_sha256']:
        raise ValueError('Candidate transform changed')
    if workspace.exists():
        raise FileExistsError('Choose a new workspace; existing runs are preserved')
    package = workspace/'src/osp_mission'
    shutil.copytree(release/'runtime/src/osp_mission', package)
    shutil.copytree(candidate/'overlay', package, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    apply(package)
    subprocess.run(['c++', '-std=c++17', '-O3', '-ffp-contract=off', '-fno-fast-math', '-shared', '-fPIC',
                    str(package/'osp_mission/raycast.cpp'), '-o', str(package/'osp_mission/_raycast.so')], check=True)
    subprocess.run(['g++', '-std=c++17', '-O1', '-I/opt/ros/jazzy/include', '-I/usr/include/eigen3',
                    str(package/'tools/export_slam_graph_poses.cpp'), '-L/opt/ros/jazzy/lib',
                    '-Wl,-rpath,/opt/ros/jazzy/lib', '-lkartoSlamToolbox', '-lboost_serialization',
                    '-lboost_thread', '-lpthread', '-o', str(package/'tools/export_slam_graph_poses')], check=True)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'ROS_PACKAGE_PATH'):
        env.pop(key, None)
    command = 'source /opt/ros/jazzy/setup.bash\ncolcon build --symlink-install --base-paths src --packages-select osp_mission\n'
    command += 'source '+shlex.quote(str(workspace/'install/setup.bash'))+'\n'
    command += "python3 -c 'from osp_mission.raycast import native_function; native_function(); from osp_mission.near_start import validate_radius; validate_radius(.30)'\n"
    if run_tests:
        command += 'python3 -m pytest -q '+shlex.quote(str(package/'test'))+'\n'
    with (workspace/'release_build.log').open('w') as stream:
        subprocess.run(['/bin/bash', '-eo', 'pipefail', '-c', command], cwd=workspace,
                       env=env, stdout=stream, stderr=stream, check=True)
    print(workspace/'install/setup.bash')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--test', action='store_true')
    args = parser.parse_args()
    build(args.workspace.resolve(), args.test)
