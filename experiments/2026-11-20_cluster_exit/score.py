"""Compare two closed-loop runs of the same world using evaluator-only coordinates."""
import bisect
import csv
import json
import math
from pathlib import Path



def read(path):
    return json.loads(path.read_text())


def closest(rows, t):
    i = bisect.bisect_left(rows,t,key=lambda row:row[0])
    return min(rows[max(0,i-1):i+1],key=lambda row:abs(row[0]-t))


def evaluate(folder):
    scene = read(folder/'scene.json')
    audit = read(folder/'audit.json')
    with (folder/'headings.csv').open() as source:
        trace = [tuple(float(row[k]) for k in ('elapsed_sim_s','x','y','yaw_rad'))
                 for row in csv.DictReader(source)]
    trace = [p for p in trace if 0 <= p[0] <= audit['elapsed_sim_s']+.001]
    poses = read(folder/'decision_poses.json')
    goals = [json.loads(l) for l in (folder/'native/center_wall.jsonl').read_text().splitlines()]
    boxes = {b['name']:(b['x'],b['y']) for b in scene['obstacles']}
    per_box = {}
    for name, xy in boxes.items():
        visits=0; near=False; dwell=0.
        for a,b in zip(trace,trace[1:]):
            distance=math.dist(a[1:3],xy)
            if distance<=1.8:
                dwell+=b[0]-a[0]
                if not near: visits+=1
                near=True
            elif distance>=2.1:
                near=False
        per_box[name]=dict(goals=[],entries=visits,dwell_1_8m_s=round(dwell,2))
    for i,g in enumerate(goals):
        if g.get('kind') != 'interior_surface' or g.get('surface') is None:continue
        s=closest(poses,g['elapsed'])
        assert abs(s[0]-g['elapsed'])<=.26
        w=closest(trace,s[0])
        angle=w[3]-s[3]; c=math.cos(angle); si=math.sin(angle)
        x,y=g['surface'][0]-s[1],g['surface'][1]-s[2]
        pos=w[1]+c*x-si*y,w[2]+si*x+c*y
        name=min(boxes,key=lambda k:math.dist(pos,boxes[k]))
        if math.dist(pos,boxes[name])<=.60: per_box[name]['goals'].append(i+1)
    return dict(run=folder.name,world=audit['world_sha256'],initial=trace[0][1:4],
                reached=audit['registered_and_within_s'],elapsed=audit['elapsed_sim_s'],
                bumper=audit['bumper_events'],boxes=per_box,
                exit_goals=[i+1 for i,g in enumerate(goals) if g.get('cluster_exit')],
                active_decisions=sum(bool(g.get('crowded_centers')) for g in goals),
                other_surface_goals=[i+1 for i,g in enumerate(goals)
                    if g.get('kind')=='interior_surface' and g.get('reason')=='inspect_other_surface'])


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    args=parser.parse_args()
    OLD,NEW=args.baseline,args.candidate
    old,new=evaluate(OLD),evaluate(NEW)
    assert old['world']==new['world'] and read(OLD/'scene.json')==read(NEW/'scene.json')
    assert math.dist(old['initial'][:2],new['initial'][:2])<.03
    assert abs(math.remainder(old['initial'][2]-new['initial'][2],math.tau))<math.radians(1.)
    out=json.dumps(dict(wall_cues=old,cluster_exit=new),indent=2)+'\n'
    print(out)
