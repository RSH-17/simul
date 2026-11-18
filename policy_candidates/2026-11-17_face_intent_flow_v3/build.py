"""Build the checked adapter chain in a fresh workspace without starting ROS."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess


def build(workspace, tests, extra_stages=()):
    candidate=Path(__file__).resolve().parent
    release=candidate.parents[1]/'policy_releases/2026-11-13_face_intent_v2'
    for relative,expected in json.loads((release/'release.json').read_text())['files'].items():
        if hashlib.sha256((release/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Frozen release changed: {relative}')
    stages=[candidate.parent/'2026-11-17_face_intent_start30_v1',
            candidate.parent/'2026-11-17_face_intent_body_scan_v2',candidate, *extra_stages]
    for stage in stages:
        manifest=json.loads((stage/'candidate.json').read_text())
        for relative,expected in manifest['overlay_files'].items():
            if hashlib.sha256((stage/'overlay'/relative).read_bytes()).hexdigest()!=expected:
                raise ValueError(f'Overlay changed: {stage.name}/{relative}')
        if hashlib.sha256((stage/'runtime_changes.py').read_bytes()).hexdigest()!=manifest['transform_sha256']:
            raise ValueError(f'Transform changed: {stage.name}')
    if workspace.exists():
        raise FileExistsError('Choose a new workspace; existing runtimes are preserved')
    package=workspace/'src/osp_mission'
    shutil.copytree(release/'runtime/src/osp_mission',package)
    for index,stage in enumerate(stages):
        shutil.copytree(stage/'overlay',package,dirs_exist_ok=True,
            ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        spec=importlib.util.spec_from_file_location(f'adapter_stage_{index}',stage/'runtime_changes.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.apply(package)
    subprocess.run(['c++','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-shared','-fPIC',
        str(package/'osp_mission/raycast.cpp'),'-o',str(package/'osp_mission/_raycast.so')],check=True)
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(package/'tools/export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib',
        '-Wl,-rpath,/opt/ros/jazzy/lib','-lkartoSlamToolbox','-lboost_serialization',
        '-lboost_thread','-lpthread','-o',str(package/'tools/export_slam_graph_poses')],check=True)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    for key in ('AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH','ROS_PACKAGE_PATH'):
        env.pop(key,None)
    command='source /opt/ros/jazzy/setup.bash\ncolcon build --symlink-install --base-paths src --packages-select osp_mission\n'
    command+='source '+shlex.quote(str(workspace/'install/setup.bash'))+'\n'
    if tests:
        command+='python3 -m pytest -q '+shlex.quote(str(package/'test'))+'\n'
    with (workspace/'release_build.log').open('w') as stream:
        subprocess.run(['/bin/bash','-eo','pipefail','-c',command],cwd=workspace,env=env,
                       stdout=stream,stderr=stream,check=True)
    print(workspace/'install/setup.bash')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--test',action='store_true')
    args=parser.parse_args();build(args.workspace.resolve(),args.test)
