"""Compare only completed, paired recordings. Include stopped/failed trials in means."""
import argparse
import json
import math
from pathlib import Path
import statistics

import yaml


def load(path):
    return json.loads(path.read_text())


def measurements(run, row):
    scene = load(run/'scene.json')
    start = yaml.safe_load((run/'scene.yaml').read_text())['start']
    metric = load(run/'metrics.json')
    by_id = {t['id']: t.get('attached_to','') for t in scene['targets']}
    if len(by_id) != 3 or '' in by_id.values():
        raise ValueError('Missing fixture truth in '+str(run))
    decoded = set(row['registered_ids'])
    near = set(map(int, metric['registered_and_within_s']))
    walls = {i for i,h in by_id.items() if h.lower().startswith('wall')}
    boxes = by_id.keys() - walls
    return dict(seed=scene['seed'],run=run.name,sha256=metric['world_sha256'],expected_ids=sorted(by_id),
                configured_start=list(map(float,start)),
                final_ids=sorted(decoded), wall_ids=sorted(walls), box_ids=sorted(boxes),
                unexpected_ids=sorted(decoded-by_id.keys()), verified_near_ids=sorted(near),
                wall_reg=len(decoded & walls), box_reg=len(decoded & boxes),
                wall_near=len(near & walls), box_near=len(near & boxes),
                strict=row['strict_saved_map_success'], map_errors_m=row['map_errors_saved_m'],
                elapsed_s=row['elapsed_sim_s'],distance_m=row['path_length_m'],bumps=row['bumper_events'],
                stopped_s=row['motion']['stopped_s'], selected_goal_counts=row['selected_goal_counts'],
                cancellations=row['cancellations'],duplicate_selections=row['consecutive_same_selected_goal_count'],
                settled_start_world_xy=row['start_world_pose'][:2],cpu=row['cpu'].get('detector'),
                camera_p95_age_s=row['camera']['processed_age_sim_p95_s'])


def read_group(root):
    if not (root/'report.json').exists():
        return {}
    paths = {Path(r).name:Path(r) for r in load(root/'runs.json')}
    return {r['seed']:measurements(paths[r['run']],r) for r in load(root/'report.json')['runs']}


def summary(rows):
    sums = ('wall_reg','box_reg','wall_near','box_near','strict','bumps','duplicate_selections')
    result = {k:sum(r[k] for r in rows) for k in sums}
    for key in ('elapsed_s','distance_m','stopped_s'):
        result['mean_'+key] = statistics.mean(r[key] for r in rows)
    result['n'] = len(rows)
    return result


def report(root):
    protocol = load(root/'protocol.json')
    baseline = read_group(Path(protocol['baseline_archive']))
    baseline.update(read_group(root/'baseline'))
    candidate = read_group(root/'candidate')
    out = {}
    for split in ('regression','heldout'):
        pairs = []
        for seed in protocol[split]:
            if seed not in baseline or seed not in candidate:
                continue
            a,b=baseline[seed],candidate[seed]
            expected_hash = protocol['worlds'][str(seed)]
            if a['sha256'] != expected_hash or b['sha256'] != expected_hash:
                raise ValueError('Different fixture in pair: '+str(seed))
            if a['configured_start'] != b['configured_start']:
                raise ValueError('Configured starting pose changed: '+str(seed))
            if a['expected_ids'] != b['expected_ids'] or a['wall_ids'] != b['wall_ids']:
                raise ValueError('Ground-truth disagreement')
            displacement = math.dist(a['settled_start_world_xy'], b['settled_start_world_xy'])
            if displacement > .15:
                raise ValueError(f'Start poses differ on {seed} by {displacement:.2f}m')
            pairs.append(dict(seed=seed,baseline=a,candidate=b,settled_start_distance_m=displacement))
        groups = {k:summary([p[k] for p in pairs]) for k in ('baseline','candidate')} if pairs else {}
        out[split] = dict(pairs=pairs,summary=groups,planned=protocol[split])
    out['complete'] = all(len(out[s]['pairs'])==len(protocol[s]) for s in ('regression','heldout'))
    return out


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    data=json.dumps(report(args.root.resolve()),indent=2)+'\n'
    if args.output:
        args.output.write_text(data)
    else:
        print(data)
