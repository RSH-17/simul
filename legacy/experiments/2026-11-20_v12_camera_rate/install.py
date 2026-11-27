"""Copy frozen v1.2 and change only simulated camera dimensions and FPS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def install(source,destination):
    if destination.exists():raise FileExistsError(destination)
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    module=package/'osp_mission/sensor_model.py'
    text=module.read_text()
    text=text.replace('    width=int(camera_width)', '''    import os
    width=int(os.environ.get('CAMERA_BENCH_WIDTH',camera_width))
    height=int(os.environ.get('CAMERA_BENCH_HEIGHT',width*3//4))
    fps=float(os.environ.get('CAMERA_BENCH_FPS','5'))
    if width not in (250,320,400,480,640) or height!=width or fps not in (5.,10.,15.):
        raise ValueError('Expected a square 250/320/400/480/640 camera at 5/10/15 FPS')''')
    text=text.replace("    if width not in (320,640,960):raise ValueError('camera_width must be 320, 640 or 960')\n",'')
    text=text.replace('str(5 if rgbd else','str(fps if rgbd else')
    text=text.replace("('height',width*3//4)","('height',height)")
    module.write_text(text)
    hashes={}
    for path in (source/'src/osp_mission').rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts or '.pytest_cache' in path.parts:continue
        relative=path.relative_to(source/'src/osp_mission')
        if str(relative)=='osp_mission/sensor_model.py':continue
        assert path.read_bytes()==(package/relative).read_bytes(),relative
        hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
    (destination/'frozen_v12_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    subprocess.run(['python3','-m','py_compile',str(module)],check=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path)
    args=parser.parse_args();install(args.source,args.destination)
