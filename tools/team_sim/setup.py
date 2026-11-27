"""Fetch identical simulation fixtures and build an independent editable policy."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]

def home():return Path(os.environ.get('TEAM_SIM_HOME',str(Path.home()/'.local/share/simul-team'))).resolve()
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def fetch(url, target):
    """Download a release asset; fall back to an authenticated `gh` for private repositories."""
    try:
        urllib.request.urlretrieve(url,target);return
    except urllib.error.HTTPError as exc:
        if exc.code not in (401,403,404):raise
    import re,shutil
    match=re.fullmatch(r'https://github\.com/([^/]+/[^/]+)/releases/download/([^/]+)/([^/]+)',url)
    gh=shutil.which('gh')
    if not match or not gh:
        raise SystemExit('Fixture download needs repository access: run `gh auth login`, or pass --archive PATH')
    repo,tag,name=match.groups()
    with tempfile.TemporaryDirectory(dir=target.parent) as tmp:
        subprocess.run([gh,'release','download',tag,'-R',repo,'-p',name,'-D',tmp],check=True)
        Path(tmp,name).replace(target)


def fixtures(archive=None):
    cfg=json.loads((HERE/'fixtures.json').read_text());cache=home();cache.mkdir(parents=True,exist_ok=True)
    destination=cache/cfg['root']
    if not destination.exists():
        download=Path(archive).resolve() if archive else cache/'fixtures-download.tar.gz'
        if not download.exists():
            temporary=download.with_suffix('.download')
            fetch(cfg['url'],temporary);temporary.replace(download)
        if digest(download)!=cfg['sha256']:raise ValueError('Fixture archive SHA-256 mismatch')
        with tarfile.open(download) as tar:
            for item in tar.getmembers():
                target=(cache/item.name).resolve()
                if not target.is_relative_to(cache) or item.issym() or item.islnk():raise ValueError('Unsafe fixture archive')
            tar.extractall(cache,filter='data')
    manifest=json.loads((destination/'manifest.json').read_text())
    for name,sha in manifest['sha256'].items():
        if digest(destination/name)!=sha:raise ValueError('Modified fixture: '+name)
    return destination

def main():
    p=argparse.ArgumentParser();p.add_argument('workspace',type=Path);p.add_argument('--policy',choices=['balanced','rescue360','adaptive-survey','object-followthrough','tiny-named-relook','surface-followthrough','balanced-print-relook','visual-survey','visual-survey-focus','marker-events','wall-slot-escape','far-wall-slot','v15257-short-lap','wall-tour','frontier-tour'],default='balanced');p.add_argument('--archive',type=Path)
    a=p.parse_args();workspace=a.workspace.expanduser().resolve()
    if workspace.exists():raise FileExistsError('Keep existing policies. Choose a new workspace.')
    workspace.parent.mkdir(parents=True,exist_ok=True)
    fixture=fixtures(a.archive)
    release=REPO/'policy_releases/2026-11-21_mixed_camera_sync_v151'
    if a.policy=='balanced':
        subprocess.run([sys.executable,str(release/'build.py'),str(workspace)],check=True)
    else:
        candidate = {'rescue360':'2026-11-21_v151_rescue360',
                     'adaptive-survey':'2026-11-22_v151_adaptive_survey',
                     'object-followthrough':'2026-11-22_v151_object_followthrough',
                     'tiny-named-relook':'2026-11-22_v151_tiny_named_relook',
                     'surface-followthrough':'2026-11-22_v151_surface_followthrough',
                     'balanced-print-relook':'2026-11-22_v151_balanced_print_relook',
                     'visual-survey':'2026-11-22_v151_visual_survey',
                     'visual-survey-focus':'2026-11-22_v152_focus_continuity',
                     'marker-events':'2026-11-26_v151_marker_events',
                     'wall-slot-escape':'2026-11-26_v154_wall_slot_escape',
                     'far-wall-slot':'2026-11-26_v155_far_wall_slot',
                     'v15257-short-lap':'2026-11-27_v15257_short_lap_import',
                     'wall-tour':'2026-11-27_v156_wall_tour',
                     'frontier-tour':'2026-11-27_v157_frontier_tour'}[a.policy]
        subprocess.run([sys.executable, str(REPO/'policy_candidates'/candidate/'install.py'),
                        str(release), str(workspace)], check=True)
        pkg=workspace/'src/osp_mission'
        subprocess.run(['c++','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-shared','-fPIC',str(pkg/'osp_mission/raycast.cpp'),'-o',str(pkg/'osp_mission/_raycast.so')],check=True)
        subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',str(pkg/'tools/export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread','-o',str(pkg/'tools/export_slam_graph_poses')],check=True)
        subprocess.run(['colcon','build','--symlink-install','--packages-select','osp_mission'],cwd=workspace,check=True)
    (workspace/'team_policy.json').write_text(json.dumps(dict(policy=a.policy,fixtures=str(fixture),source='frozen-v1.5.1' if a.policy!='v15257-short-lap' else 'release-policy-v1.5.2.57',editable_source=str(workspace/'src/osp_mission/osp_mission')),indent=2)+'\n')
    print('Prepared',workspace,'with',fixture)

if __name__=='__main__':main()
