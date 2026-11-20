"""After-run scoring only. Simulator truth never enters the navigation process."""
import bisect
import csv
import json
import math
from pathlib import Path

SURFACE_GATE = .50

def read(p):
    return json.loads(p.read_text())

def lines(p):
    return [json.loads(line) for line in p.read_text().splitlines()]

def trace(p):
    with p.open() as f:
        return [[float(r[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')] for r in csv.DictReader(f)]

def near_time(seq, t):
    i=bisect.bisect_left(seq,t,key=lambda r:r[0])
    return min(seq[max(0,i-1):i+1],key=lambda row:abs(row[0]-t))

def to_world(point, truth, snap):
    a=truth[3]-snap[3]; x=point[0]-snap[1];y=point[1]-snap[2]
    return [truth[1]+math.cos(a)*x-math.sin(a)*y, truth[2]+math.sin(a)*x+math.cos(a)*y]

def examine(folder, baseline):
    audit=read(folder/'audit.json')
    scene=read(folder/'scene.json')
    wall=next(t for t in scene['targets'] if t['attached_to'].startswith('wall'))
    wp=wall['x'],wall['y']
    goals=lines(folder/'native/center_wall.jsonl')
    hyb=audit['hybrid_goal_types']
    assert len(hyb)==len(goals)
    total=audit['elapsed_sim_s']
    history=[r for r in trace(folder/'headings.csv') if 0<=r[0]<=total+.001]
    # Verify independently from registered IDs in the truth stream itself.
    registered_near={}
    targets={t['id']:(t['x'],t['y']) for t in scene['targets']}
    with (folder/'trajectory.csv').open() as stream:
        for i,row in enumerate(csv.DictReader(stream)):
            if i>=audit['truth_samples']: break
            for ident in map(int,row['registered_ids'].split()):
                if ident not in registered_near and math.dist((float(row['world_x']),float(row['world_y'])),targets[ident])<=1.5:
                    registered_near[ident]=float(row['elapsed_sim_s'])
    assert set(registered_near)==set(map(int,audit['registered_and_within_s']))
    for ident,val in registered_near.items():
        assert abs(val-audit['registered_and_within_s'][str(ident)])<=.022
    poses=[] if baseline else read(folder/'decision_poses.json')
    looks=[] if baseline or not (folder/'native/face_directed_looks.jsonl').exists() else lines(folder/'native/face_directed_looks.jsonl')
    ends=[g['elapsed'] for g in goals[1:]]+[total]
    boxes=[(i,g) for i,g in enumerate(goals) if g.get('batch_box') is not None]
    evidence=[]
    for i,(g,end) in enumerate(zip(goals,ends)):
        if not g.get('weak_wall') and hyb[i]!='print': continue
        p=g.get('surface') if g.get('weak_wall') else g.get('print_point')
        assert p is not None and poses
        t=g['elapsed'];snap=near_time(poses,t)
        assert abs(snap[0]-t)<.26, (folder,t,snap)
        world=to_world(p,near_time(history,snap[0]),snap)
        nearest=min(targets,key=lambda ident:math.dist(targets[ident],world))
        err=math.dist(wp,world)
        kind='weak' if g.get('weak_wall') else 'print'
        marker=wall['id'] if kind=='weak' else nearest
        where=targets[marker]; err_expected=math.dist(where,world)
        j=i+1
        while j<len(goals) and hyb[j] in ('focus','approach') and goals[j].get('marker_id')==marker:
            j+=1
        chain_end=ends[j-1]
        segment=[r for r in history if t<=r[0]<=end+.001]
        chain=[r for r in history if t<=r[0]<=chain_end+.001]
        direct_min=min(math.dist(r[1:3],where) for r in segment)
        chain_min=min(math.dist(r[1:3],where) for r in chain)
        vpoint=to_world(g['selected_xy'],near_time(history,snap[0]),snap)
        prevbox=[(idx,box) for idx,box in boxes if idx<i]
        look_evidence=[]
        for idx,box in prevbox:
            if any(l['event']=='consumed' and box['elapsed']-.001<=l['stamp']-audit['simulation_start_clock']<ends[idx]+.001 for l in looks):
                look_evidence.append(idx+1)
        starts=[r[0] for support in g.get('weak_support',[]) for r in support]
        initial=audit['simulation_start_clock']
        support_times = {round(stamp-initial, 3) for stamp in starts}
        during_box = {stamp for stamp in support_times if any(
            box['elapsed'] <= stamp < ends[idx] for idx,box in prevbox)}
        credit=registered_near.get(marker)
        evidence.append(dict(goal=i+1,kind=kind,wall_id=wall['id'],marker_id=marker,
            marker_registered_before=marker in g.get('confirmed_ids',[]),
            observer_registration_s=audit['registered_s'].get(str(marker)),
            elapsed=t,end=end,chain_end=chain_end,duration_s=end-t,
            first_support_s=min(starts)-initial if starts else None,
            last_support_s=max(starts)-initial if starts else None,
            prior_box_goals=len(prevbox),prior_box_looks=look_evidence,
            supporting_frames=len(support_times),supporting_frames_during_box=len(during_box),
            world_surface=world,wall_error_m=err,nearest_marker=nearest,
            nearest_error_m=math.dist(targets[nearest],world),expected_error_m=err_expected,
            correct=err_expected<=SURFACE_GATE,
            start_distance_m=math.dist(near_time(history,t)[1:3],where),
            waypoint_min_m=min(math.dist(r[1:3],vpoint) for r in segment),
            direct_min_m=direct_min,chain_min_m=chain_min,approach_s=credit,
            chain_registered_near=bool(err_expected<=SURFACE_GATE and credit is not None and t<=credit<=chain_end+.021)))
    return dict(run=folder.name,seed=scene['seed'],world=audit['world_sha256'],wall_id=wall['id'],
                start=history[0][1:],registered=audit['final_ids'],approached=sorted(registered_near),
                wall_success=wall['id'] in registered_near,elapsed=total,bumper=audit['bumper_events'],
                registered_near=registered_near,
                advisory_stable_peak=max((g.get('stable_unused',0) for g in goals),default=0),
                total_box_goals=len(boxes),evidence=evidence,
                wrong_s=sum(e['duration_s'] for e in evidence if not e['correct']),
                wrong_count=sum(not e['correct'] for e in evidence))

def pair(old, folder):
    before,after=examine(old,True),examine(folder,False)
    assert before['world']==after['world'] and read(old/'scene.json')==read(folder/'scene.json')
    assert (old/'scene.yaml').read_bytes()==(folder/'scene.yaml').read_bytes()
    a,b=before['start'],after['start']
    assert math.dist(a[:2],b[:2])<.03 and abs(math.remainder(a[2]-b[2],math.tau))<math.radians(1)
    return dict(before=before,after=after)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('box_batch_trial', type=Path)
    parser.add_argument('wall_cues_trial', type=Path)
    args = parser.parse_args()
    print(json.dumps(pair(args.box_batch_trial, args.wall_cues_trial), indent=2))
