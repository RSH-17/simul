"""Generate or reuse a portable random exam world with its original physical shell."""
import fcntl
from pathlib import Path
import secrets
import sys
import tempfile

import numpy as np

EXAM = Path(__file__).resolve().parents[2]/'experiments/2026-11-22_random_exam'
sys.path.insert(0,str(EXAM))
from fixture import build, digest, read, save


def selection(seed, random_layout, markers, legacy):
    if seed is not None and not 0 <= seed < 2**32:
        raise ValueError('A seed must be a 32-bit unsigned integer')
    randomized = random_layout or seed not in legacy or markers == 'unknown'
    seed = secrets.randbits(32) if seed is None else seed
    public_count = 0 if markers == 'unknown' or (randomized and markers is None) else 3
    return seed, randomized, public_count


def fixture_for(cache, assets, seed, public_count):
    generator = (digest(EXAM/'geometry.py')[:12]+'-'+digest(EXAM/'fixture.py')[:12])
    group = cache/'random-exam'/generator/('count-unknown' if public_count==0 else 'count-three')
    group.mkdir(parents=True,exist_ok=True)
    dest = group/f'map-{seed}'
    with (group/f'{seed}.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if not dest.exists():
            number = public_count or int(np.random.default_rng([seed,0x544234]).integers(1,6))
            # The SDF contains a relocatable texture URI. An interrupted build
            # cannot leave a half-written fixture for another policy to consume.
            with tempfile.TemporaryDirectory(dir=group) as tmp:
                stage = Path(tmp)/'fixture'
                metadata = build(assets/'maps/map-112601',stage,seed,number,'shuffle')
                save(stage/'fixture.json',metadata)
                stage.rename(dest)
        manifest = read(dest/'fixture.json')
        if (manifest['seed']!=seed or manifest['tables']!='shuffle' or
                any(digest(dest/key)!=value for key,value in manifest['files'].items())):
            raise ValueError('The cached randomized fixture changed: '+str(dest))
    return dest


def finish(run):
    """Apply the native exam's independent scorer to a portable randomized recording."""
    from scoring import score
    sys.path[:0]=[str(EXAM.parent/'2026-11-22_exam_track_validation'),
                  str(EXAM.parent/'2026-11-22_v151_adaptive_survey/validation')]
    from paired import after_start_ids, errors, start_pose, XY_TOL, YAW_TOL
    from validate import measure
    config=read(run/'protocol.json')
    metrics=read(run/'metrics.json')
    scene=read(run/'scene.json')
    world={'files':{'world/osp_studio.sdf':config['effective_world_sha256']}}
    observed=measure(run,world)
    roster={target['id'] for target in scene['targets']}
    prestart,false=after_start_ids(run,roster)
    _,actual=start_pose(run)
    start=[scene['robot_spawn'][key] for key in ('x','y','yaw')]
    xy,yaw=errors(actual,start)
    movie=read(run/'audit.json')['video']
    observed.update(valid=bool(observed['valid'] and not prestart and xy<=XY_TOL and yaw<=YAW_TOL and
                               digest(run/'mission.mp4')==movie['sha256']),
                    false_registered_ids_after_start=false)
    result=score(observed,metrics,scene,duration=config['duration_sim_s'])
    result.update(start_position_error_m=xy,start_yaw_error_rad=yaw,prestart_ids=prestart)
    save(run/'exam-score.json',result)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description='Audit a completed team random exam')
    p.add_argument('recording', type=Path)
    finish(p.parse_args().recording)
