"""Audit repeated travel and new camera opportunities using *recorded* simulation data."""
import argparse
from itertools import combinations
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from geometry import intersects, polygon

STEP = .25
RADIUS = .48
MAX_AGE = 85.
MIN_AGE = 14.
NO_GAIN_S = 20.


def boundary_distance(points, corners):
    start = corners[None, :, :]
    edge = (np.roll(corners, -1, axis=0) - corners)[None, :, :]
    delta = np.asarray(points)[:, None, :] - start
    fraction = np.clip((delta*edge).sum(axis=2)/(edge*edge).sum(axis=2), 0., 1.)
    return np.linalg.norm(delta - fraction[:, :, None]*edge, axis=2).min(axis=1)


def nearby_pairs(scene):
    pairs = []
    for a, b in combinations(scene['obstacles'], 2):
        if a['object_id'] == b['object_id']:
            continue
        A, B = polygon(a), polygon(b)
        gap = 0. if intersects(A, B) else float(min(boundary_distance(A,B).min(), boundary_distance(B,A).min()))
        if gap <= .60:
            pairs.append((a['name'], b['name'], gap, A, B))
    return pairs


def resample(track):
    """Sample motion by distance, splitting over discontinuities in the pose record."""
    track = np.asarray(track)[:, :3]
    delta = np.linalg.norm(np.diff(track[:, 1:], axis=0), axis=1)
    dt = np.diff(track[:,0])
    breaks = np.flatnonzero((delta > 1.5) | (dt <= 0.) | (dt > .55)) + 1
    for segment in np.split(track, breaks):
        if len(segment) < 3:
            continue
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(segment[:,1:],axis=0),axis=1))]
        # Find the arrival time of each traversed interval. Stop time must not
        # be interpolated across neighbouring spatial samples.
        target = np.arange(STEP, length[-1]+1.e-9, STEP)
        if not len(target):
            continue
        hi = np.minimum(np.searchsorted(length, target, side='left'), len(length)-1)
        lo = hi-1
        frac = ((target-length[lo])/(length[hi]-length[lo]))[:, None]
        value = segment[lo] + frac*(segment[hi]-segment[lo])
        yield np.vstack((segment[0], value))


def motion_repeats(path, max_age=MAX_AGE):
    """Classify matched directed segments after a real excursion; ignore isolated crossings."""
    time = path[:,0]
    xy = path[:,1:]
    delta = np.diff(xy, axis=0, prepend=xy[:1])
    norm = np.maximum(1.e-9, np.linalg.norm(delta,axis=1))
    tangent = delta/norm[:, None]
    out = np.zeros(len(xy), dtype=np.int8)
    for i in range(2, len(xy)):
        age = time[i]-time[:i]
        old = np.flatnonzero((age>=MIN_AGE) & (age<=max_age) & ((i-np.arange(i))*STEP>=3.)
                             & (np.linalg.norm(xy[:i]-xy[i],axis=1)<=RADIUS))
        if not len(old):
            continue
        dot = tangent[old] @ tangent[i]
        for j in np.argsort(-np.abs(dot),kind='stable'):
            if abs(dot[j]) < math.cos(math.radians(40.)):
                break
            if np.linalg.norm(xy[old[j]:i]-xy[i],axis=1).max() >= 1.05:
                out[i] = 1 if dot[j] >= 0 else -1
                break
    changes = np.flatnonzero(np.diff(np.r_[False, out!=0, False])).reshape(-1,2)
    for start, end in changes:
        if end-start < 3:
            out[start:end]=0
    return out


def first_complete(stamps):
    """First timestamp of three distinct usable exposures, at least .15 s apart in 1 s."""
    if len(stamps)<3:
        return None
    first = np.arange(len(stamps))
    middle = np.searchsorted(stamps, stamps+.15-1.e-8)
    first = first[middle < len(stamps)]
    end = np.searchsorted(stamps, stamps[middle[first]]+.15-1.e-8)
    valid = end < len(stamps)
    end, first = end[valid], first[valid]
    valid_end = end[stamps[end] <= stamps[first]+1.+1.e-8]
    return float(stamps[valid_end.min()]) if len(valid_end) else None


def exposure_gains(scene, world, track, exposure_times, calibration):
    from surface_audit import blockers, camera_poses, faces, mountable, readable_indices
    times, origins, axes = camera_poses(track, exposure_times, calibration)
    size, height = .20,.32
    solids = blockers(world,min(origins[0,2],height-size/2),max(origins[0,2],height+size/2))
    gains = []
    for host, name, shape, a, b, offset in faces(scene):
        length = float(np.linalg.norm(b-a))
        available = length-1.3*size
        if available <= 0:
            continue
        tangent = np.r_[(b-a)/length,0.]
        normal = np.array([-tangent[1],tangent[0],0.])
        n = math.ceil(available/.4)
        for i in range(n):
            center = np.r_[a,height]+tangent*(.65*size+available*(i+.5)/n)+(offset+.004)*normal
            if not mountable(center,tangent,size,normal,solids):
                continue
            indices,_ = readable_indices(center,normal,tangent,size,18.,origins,axes,calibration,solids)
            first = first_complete(times[indices])
            if first is not None:
                gains.append(dict(t=first, name=name, host=host, kind='wall' if shape=='wall' else 'obstacle',
                                  sample=i, span_m=available/n))
    return gains


def counts(chunks,max_age,gains):
    times = np.sort(np.asarray(gains,dtype=float))
    out = dict(sampled_m=0., repeated_m=0., same_direction_m=0., opposite_direction_m=0.,
               repeated_no_new_view_m=0., cluster_visit_m=0., cluster_repeat_m=0.,cluster_repeat_no_new_view_m=0.,
               longest_continuous_repeat_m=0., intervals=[])
    for chunk, cluster in chunks:
        tags = motion_repeats(chunk,max_age)
        repeated = tags != 0
        t = chunk[:,0]
        dry = np.searchsorted(times,t,side='right')==np.searchsorted(times,t-NO_GAIN_S,side='left')
        out['sampled_m'] += (len(t)-1)*STEP
        for field, mask in [('repeated_m',repeated),('same_direction_m',tags==1),
                            ('opposite_direction_m',tags==-1),('repeated_no_new_view_m',repeated&dry),
                            ('cluster_visit_m',cluster),('cluster_repeat_m',repeated&cluster),
                            ('cluster_repeat_no_new_view_m',repeated&cluster&dry)]:
            out[field]+=float(np.count_nonzero(mask[1:]))*STEP
        bounds=np.flatnonzero(np.diff(np.r_[False,repeated,False])).reshape(-1,2)
        for a,b in bounds:
            length=(b-a)*STEP
            out['longest_continuous_repeat_m']=max(length,out['longest_continuous_repeat_m'])
            out['intervals'].append(dict(start_s=round(float(t[max(0,a-1)]),2),end_s=round(float(t[b-1]),2),
                                         length_m=length, no_new_view_m=STEP*int(dry[a:b].sum()),
                                         near_cluster_m=STEP*int(cluster[a:b].sum())))
    return out


def analyze(folder, board_root, calibration, sensor_result):
    """Use only final bags with verified timestamps; absent sensor data must stay unknown."""
    from sensor_audit import bag_messages, epochs
    if not sensor_result['valid']:
        raise ValueError('Incomplete camera/clock evidence for '+folder.name)
    read = lambda p: json.loads(p.read_text())
    metrics=read(folder/'metrics.json')
    events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines() if line.strip()]
    start, = [e['sim'] for e in events if e['event']=='start_ack']
    evidence, = epochs(bag_messages(board_root/'sensor-recordings'/folder.name))
    end=start+min(600.,metrics['elapsed_sim_s'])
    stamps=np.unique([round(when-start,8) for when,_ in evidence['camera'] if start<=when<=end])
    if abs(len(stamps)-sensor_result['processed_stamps'])>1:
        raise ValueError('Camera ledger differs from sensor audit')
    track=np.loadtxt(folder/'headings.csv',skiprows=1,delimiter=',',ndmin=2)
    scene=read(folder/'scene.json')
    world=ET.parse(folder/'world/osp_studio.sdf').find('world')
    sessions=exposure_gains(scene,world,track,stamps,calibration)
    ids=set()
    gain_times=[g['t'] for g in sessions]
    for e in events:
        if e['event']=='ids' and set(e.get('ids',()))-ids:
            ids.update(e['ids'])
            gain_times.append(e['sim']-start)
    for kind in ('wall','obstacle'):
        measured=sum(g['span_m'] for g in sessions if g['kind']==kind)
        expected=sensor_result['surface_coverage'][kind]['observed_span_m']
        if not math.isclose(measured,expected,abs_tol=1.e-5):
            raise ValueError(f'First camera views disagree with surface audit: {kind} {measured} {expected}')
    pairs=nearby_pairs(scene)
    chunks=[]
    for line in resample(track[(track[:,0]>=0)&(track[:,0]<=end-start)]):
        xy=line[:,1:]
        cluster=np.zeros(len(line),dtype=bool)
        for _,_,_,a,b in pairs:
            cluster |= (boundary_distance(xy,a)<=2.)&(boundary_distance(xy,b)<=2.)
        chunks.append((line,cluster))
    goal_file=folder/'native/center_wall.jsonl'
    goals=[json.loads(row) for row in goal_file.read_text().splitlines() if row.strip()] if goal_file.exists() else []
    route_policy={k:sum(bool(g.get(k)) for g in goals) for k in ('survey_escape','survey_wall','cluster_exit')}
    route_policy['distant_hint']=sum(g.get('distant_hint_id') is not None for g in goals)
    return dict(run=folder.name, seed=scene['seed'], distinct_objects=len({b['object_id'] for b in scene['obstacles']}),
                nearby_pairs=[dict(a=a,b=b,gap_m=d) for a,b,d,_,_ in pairs],
                since_first_view_s=NO_GAIN_S, recent=counts(chunks,MAX_AGE,gain_times),
                entire_mission=counts(chunks,600.,gain_times),policy_decisions=route_policy,
                first_observations=sessions)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for n in ('panel','recordings','calibration'):
        parser.add_argument(n,type=Path)
    args=parser.parse_args()
    target=args.panel/'route-repeats'
    target.mkdir(exist_ok=True)
    cal=json.loads(args.calibration.read_text())
    for row in json.loads((args.panel/'episodes.json').read_text()):
        name=row['name']
        sensor=args.panel/'sensor-audit'/(name+'.json')
        dest=target/(name+'.json')
        if dest.exists() or not sensor.is_file():
            continue
        result=analyze(args.recordings/name,args.panel,cal,json.loads(sensor.read_text()))
        tmp=dest.with_suffix('.new')
        tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        tmp.replace(dest)
        print(name, result['recent']['cluster_visit_m'],result['recent']['repeated_no_new_view_m'],flush=True)


if __name__=='__main__':
    main()
