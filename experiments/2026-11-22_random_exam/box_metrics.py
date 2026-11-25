"""Compare recorded wall and box search, counting each fresh exam world once."""
import argparse
import math
from pathlib import Path
from statistics import mean

from export import read, sha, write
from geometry import wall_segments
from visual_survey_results import endpoint, matched


ROLES=('baseline','candidate')


def outer_walls(scene, coverage):
    """Length of observable marker-centre intervals; the south recess belongs to the south wall."""
    settings=coverage['parameters']
    groups=coverage['groups']['wall']['faces']
    sides={s:dict(observed_m=0.,mountable_m=0.) for s in ('north','east','south','west')}
    for name,a,b in wall_segments(scene):
        if name.startswith('structure_'):
            continue
        side='south' if name.startswith('south_') else name
        span=max(0.,math.dist(a,b)-1.3*settings['print_m'])
        if not span:continue
        unit=span/math.ceil(span/settings['sample_interval_max_m'])
        face=groups[name]
        sides[side]['mountable_m']+=unit*face['sampled_centers']
        sides[side]['observed_m']+=unit*face['counts'].get('observed',0)
    return sides


def comparison(root):
    plan=read(root/'protocol.json')
    reg=read(root/'registration.json')
    if (sha(root/'registration.json')!=plan['registration_sha256'] or reg['furniture']!='fixed' or
            reg['known_marker_count']!=3 or reg['limits'].get('obstacle_geometry')!='original_exam_boxes'):
        raise ValueError('Expected the registered five-box exam')
    for world in plan['worlds']:
        shapes=world['object_shapes']
        if world['movable_objects']!=5 or len(shapes)!=5 or set(shapes.values())!={'box'}:
            raise ValueError('A trial has different obstacles')
    ledger=read(root/'comparison.json')
    scheduled={t['pair']:t for t in plan['plan']}
    rows=[]
    for pair in ledger['pairs']:
        if not pair['valid']:continue
        task=scheduled[pair['pair']]
        world=plan['worlds'][task['world']]
        if (pair['seed']!=world['seed'] or bool(pair['repeat'])!=bool(task['repeat']) or
                any(pair[r]['world_sha256']!=world['files']['world/osp_studio.sdf'] or
                    pair[r]['source_sha256']!=reg[r]['source_manifest_sha256'] for r in ROLES)):
            raise ValueError('World or controller differs from its registration')
        a,b=(endpoint(root,pair[r]) for r in ROLES)
        matched(a,b)
        if a['sensor_valid'] and b['sensor_valid']:
            scene=read(root/'fixtures'/f"map-{world['seed']}"/'scene.json')
            for entry in (a,b):
                sensor=read(root/'sensor-audit'/(entry['run']+'.json'))
                entry['outer_walls']=outer_walls(scene,sensor['surface_coverage'])
        rows.append(dict(pair=pair['pair'],seed=world['seed'],repeat=bool(task['repeat']),baseline=a,candidate=b))
    fresh=[p for p in rows if not p['repeat']]
    if len({p['seed'] for p in fresh})!=len(fresh):
        raise ValueError('Duplicate independent world')
    return dict(planned_pairs=len(scheduled),valid_pairs=len(rows),distinct_worlds=len(fresh),
                near_and_completion=ledger['first_runs'],camera_and_routes=paired_camera_summary(fresh),
                diagnostic=reg.get('cluster_start'),pairs=rows)


def paired_camera_summary(pairs):
    camera=[p for p in pairs if all(p[r]['sensor_valid'] for r in ROLES)]
    route=[p for p in camera if all(p[r]['repeat'] for r in ROLES)]
    def camera_mean(role,field):
        values=[p[role][field] for p in camera if all(p[r][field] is not None for r in ROLES)]
        return dict(pairs=len(values),mean=mean(values) if values else None)
    def report(role):
        metrics={f:camera_mean(role,f) for f in ('wall_fraction','object_fraction')}
        metrics.update(boxes_all_four=sum(p[role]['boxes_four'] for p in camera),
                       boxes=sum(p[role]['boxes'] for p in camera))
        sides={}
        for p in camera:
            for name,surface in p[role].get('outer_walls',{}).items():
                tally=sides.setdefault(name,dict(observed_m=0.,mountable_m=0.))
                for key in tally:tally[key]+=surface[key]
        metrics['outer_walls']={s:dict(v,fraction=v['observed_m']/v['mountable_m'] if v['mountable_m'] else None)
                                for s,v in sides.items()}
        for name in ('repeated_m','repeated_no_new_view_m','cluster_visit_m','cluster_repeat_no_new_view_m'):
            values=[p[role]['repeat']['recent'][name] for p in route]
            metrics[name]=dict(pairs=len(values),mean=mean(values) if values else None,total=sum(values) if values else None)
        return metrics
    return dict(matched_camera_pairs=len(camera),matched_route_pairs=len(route),**{r:report(r) for r in ROLES})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('panel',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    write(args.output,comparison(args.panel))
