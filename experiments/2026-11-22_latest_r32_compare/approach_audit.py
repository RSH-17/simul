"""Reconcile terminal status and saved high-rate Gazebo poses for a paired export."""
import argparse
import csv
import json
import math
from pathlib import Path


def replay(run, summary):
    metrics = json.loads((run / 'metrics.json').read_text())
    targets = {t['id']: (t['x'], t['y']) for t in json.loads((run / 'scene.json').read_text())['targets']}
    registered = {int(i): t for i, t in metrics['registered_s'].items()}
    end = metrics['elapsed_sim_s']
    deadline = 600.
    end_pose = None
    last = {}
    ever = {}
    witnesses = {}
    extra_bump = False
    with (run / 'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            t = float(row['elapsed_sim_s'])
            if not 0 <= t <= min(end + .25, deadline) + 1e-8:
                continue
            point = (float(row['world_x']), float(row['world_y']))
            current = set(map(int, row['registered_ids'].split()))
            if t <= end + 1e-8:
                end_pose = point
            elif end_pose is None or math.dist(point, end_pose) > .02:
                continue
            if t > end and row['bumper'] == 'True':
                extra_bump = True
            for i, target in targets.items():
                d = math.dist(point, target)
                last[i] = dict(t=t, distance_m=d)
                if d <= 1.5:
                    ever.setdefault(i, t)
                # The pose must carry the registered ID and its sensor timestamp
                # must not predate the first status notification that registered it.
                if i in current and i in registered and t + 1e-8 >= registered[i] and d <= 1.5:
                    witnesses.setdefault(i, t)
    expected = set(targets)
    observed = set(witnesses)
    fixed_map = summary['saved_map_error_m']
    missing_fresh_pose = [i for i in expected - observed if i in registered and i in last
                          and 0 < registered[i] - last[i]['t'] <= .25
                          and abs(registered[i] - end) < 1e-6]
    static_ok = (set(summary['registered']) == expected and not summary['unexpected_ids']
                 and set(map(int, fixed_map)) == expected and max(fixed_map.values()) <= .25
                 and not metrics['bumper_events'] and not extra_bump)
    if static_ok and observed == expected:
        strict = 'confirmed'
    elif static_ok and expected - observed == set(missing_fresh_pose):
        strict = 'unresolved_terminal_sample'
    else:
        strict = 'failed'
    status = metrics['last_policy_status']
    return dict(run=run.name, strict=strict, original_common_success=summary['common_success'],
                registered_s=registered, sensor_stamped_after_registration_within_s=witnesses,
                anytime_within_s=ever, last_valid_pose=last,
                unresolved_terminal_ids=missing_fresh_pose, end_s=end,
                extra_bump=extra_bump, terminal_reason=metrics['reason'],
                health_problem=status.get('health_problem'), health_pauses=status.get('health_pauses'),
                navigation_retry_reason=status.get('navigation_retry_reason'))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('comparison', type=Path)
    p.add_argument('recordings', type=Path)
    p.add_argument('out', type=Path)
    args = p.parse_args()
    comparison = json.loads(args.comparison.read_text())
    result = dict(protocol='Same post-hoc audit of all 15 raw trajectories. '
                  'Pose stamp at/after ID registration, pose contains registered ID, distance <=1.5 m. '
                  'Post-terminal samples allowed only within 0.25 simulated seconds, 0.02 m of last pre-end '
                  'position and the original 600 s deadline. No pose imputation. Fixed original map and bumper gates.',
                  pairs=[])
    for pair in comparison['pairs']:
        audit = {'seed': pair['seed']}
        for policy in ('balanced', 'rescue360', 'latest_r32'):
            summary = pair[policy]
            audit[policy] = replay(args.recordings / summary['run'], summary)
        result['pairs'].append(audit)
    args.out.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
