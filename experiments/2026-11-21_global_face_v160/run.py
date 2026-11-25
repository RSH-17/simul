"""Serial matched Gazebo trials; process locks also protect existing dashboard work."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
from fixtures import CASES,prepare
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-21_camera_aware_policy'))
from campaign import execute,WORKSPACES
from coverage_metrics import evaluate

def save(path,data):
    temp=path.with_suffix('.new')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temp.replace(path)

def run(root):
    assert os.environ['ROS_DOMAIN_ID']=='174' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    root.mkdir(exist_ok=False)
    WORKSPACES.update(mixed153=Path.home()/'flow_v153_region_wall_ws',global160=Path.home()/'flow_v160_global_face_r3_ws')
    os.environ['CAMERA_BENCH_TRIAL_SCRIPT']=str(HERE/'coverage_observer.py')
    manifests={k:json.loads((WORKSPACES[k]/name).read_text()) for k,name in (
        ('mixed153','region_manifest.json'),('global160','coverage_manifest.json'))}
    seeds={key:116001+i for i,key in enumerate(CASES)}
    hashes={k:prepare(root,k,s) for k,s in seeds.items()}
    save(root/'protocol.json',dict(seeds=seeds,sdf_sha256=hashes,duration_s=180,camera=[480,480,5],
         stages=manifests,limits='one trace per policy/case; common strict-stop judge',physical_drive=False))
    locks=[open(Path.home()/'track-recordings'/n,'a') for n in ('.domain.lock','.second-domain.lock')]
    print('Waiting for existing simulation to complete',flush=True)
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX)
    runs=[]
    for i,(case,seed) in enumerate(seeds.items()):
        for variant in (('global160','mixed153') if i%2==0 else ('mixed153','global160')):
            sequence=f'{case}-{root.name.split("-")[-1]}'
            folder=execute(root,variant,'balanced',seed,sequence,180)
            sdf=root/f'map-{seed}/world/osp_studio.sdf'
            assert hashlib.sha256(sdf.read_bytes()).hexdigest()==hashes[case]
            runs.append(str(folder));save(root/'runs.json',runs)
            save(folder/'coverage_comparison.json',evaluate(folder))
            save(root/'results.json',[json.loads((Path(x)/'coverage_comparison.json').read_text()) for x in runs])
    save(root/'complete.json',dict(completed=True,pairs=6,runs=len(runs)))

if __name__=='__main__':run(Path(sys.argv[1]).resolve())
