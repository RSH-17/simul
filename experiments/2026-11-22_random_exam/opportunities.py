"""Post-run audit of recorded camera opportunities, distinct from marker credit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-22_v151_adaptive_survey/development'))
from exposure_opportunity import measure


def read(path):
    return json.loads(path.read_text())


def analyze(folder, score, calibration):
    metrics=read(folder/'metrics.json')
    sensed=metrics['camera']['camera_info']
    if (sensed['width'],sensed['height'])!=(calibration['width'],calibration['height']):
        raise ValueError('Camera dimensions differ from measured calibration')
    if any(abs(a-b)>1e-5 for a,b in zip(sensed['k'],calibration['k'],strict=True)):
        raise ValueError('Recorded camera intrinsics differ from calibration')
    projection,_=measure(folder,calibration,figure_id=None)
    offline=read(folder/'audit.json')['offline_rgb']
    targets={}
    for ident, observation in projection['targets'].items():
        target=score['records'][ident]
        decoded=int(observation['online_decoded_count'])
        readable=int(observation['five_rays_clear_18px_frames'])
        if target['verified_s'] is not None:
            stage='verified'
        elif target['registered_s'] is not None:
            stage='registered_without_both_near_witnesses'
        elif decoded:
            stage='decoded_without_registration'
        elif readable:
            stage='predicted_readable_view_without_online_decode'
        elif observation['small_6_to_lt18_px']['clear_all']:
            stage='only_a_small_predicted_view'
        else:
            stage='no_clear_predicted_view_at_6px'
        targets[ident]=dict(host=target['host'],stage=stage,geometric_18px_frames=readable,
                            geometric_18px_near_frames=observation['five_rays_clear_18px_within_1p5m_frames'],
                            geometric_6_to_lt18px_frames=observation['small_6_to_lt18_px']['clear_all'],
                            geometric_first_18px_s=observation['first_5ray_18px_s'],
                            online_decodes=decoded,offline_jpeg_decodes=offline.get(ident,{}).get('frames',0),
                            near_s=observation['physical_near_s'],
                            detector_breakdown={k:v for k,v in metrics['last_detector_status'].get('by_id',{}).get(ident,{}).items()
                                                if k in ('small','border','no_range','horizontal','inconsistent','accepted')})
    goals=folder/'native/hybrid_goals.jsonl'
    selected=[json.loads(line) for line in goals.read_text().splitlines() if line.strip()] if goals.exists() else []
    prior=[];repeats=0
    for goal in selected:
        if all(isinstance(goal.get(k),(int,float)) for k in ('x','y')):
            xy=(goal['x'],goal['y'])
            if any(math.dist(xy,point)<=.35 and goal['kind']==kind for point,kind in prior):
                repeats+=1
            prior.append((xy,goal['kind']))
    return dict(run=folder.name,evidence=projection['source_sha256'],targets=targets,
                selected_hybrid_goals=len(selected),repeated_same_kind_within_35cm=repeats,
                stationary_s_below_2cm_per_s=metrics['stationary_time_s'],
                note='Camera exposure projection uses the measured optical transform and five rays. The saved JPEG and online decoder counts measure different things. Repeated goal locations can include purposeful returns with a different heading.')


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('exam',type=Path)
    cli.add_argument('recordings',type=Path)
    cli.add_argument('--calibration',type=Path,default=HERE.parent/'2026-11-21_v151_track5_boxes5/camera_geometry.json')
    args=cli.parse_args()
    calibration=read(args.calibration)
    dst=args.exam/'camera-opportunities';dst.mkdir(exist_ok=True)
    for row in read(args.exam/'episodes.json'):
        path=dst/(row['name']+'.json')
        if path.exists():continue
        record=analyze(args.recordings/row['name'],row['score'],calibration)
        record['calibration_sha256']=hashlib.sha256(args.calibration.read_bytes()).hexdigest()
        tmp=path.with_suffix('.new');tmp.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n');tmp.replace(path)


if __name__=='__main__':main()
