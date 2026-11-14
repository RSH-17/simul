"""Check the queued pocket fix through edge generation, memory and selection.

Only observed planner inputs are read. Recorded eligible choices reproduce the
original ranking. Counterfactual choices have no complete recorded memory;
their filtering is therefore checked under explicit memory stress cases, not
presented as an exact mission replay or predicted native departure.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(choices):
    return np.asarray([[c['x'], c['y'], c['yaw'], c.get('scan_angle', 0.),
                        *c['features']] for c in choices]).reshape(-1, 9)


def audit(episode, baseline, candidate):
    old = baseline / 'src/osp_mission/osp_mission'
    new = candidate / 'src/osp_mission/osp_mission'
    old_hash = {p.name: digest(p) for p in old.glob('*.py')}
    new_hash = {p.name: digest(p) for p in new.glob('*.py')}
    if {k for k in old_hash.keys() | new_hash.keys()
            if old_hash.get(k) != new_hash.get(k)} != {'start_waypoints.py'}:
        raise ValueError('Require the isolated queued pocket change')
    sys.path.insert(0, str(old.parent))
    from osp_mission import edge_policy
    from osp_mission.exploration_memory import ExplorationMemory, transform_pose
    if Path(edge_policy.__file__).resolve() != (old/'edge_policy.py').resolve():
        raise ValueError('Wrong frozen planner imported')
    spec = importlib.util.spec_from_file_location('queued_pocket_selection', new/'start_waypoints.py')
    revised = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(revised)
    diagnostics = [json.loads(line) for line in (episode/'planning_diagnostics.jsonl').read_text().splitlines()]
    memory = [json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
    rows = []
    original_start = edge_policy.start_options
    for file in sorted((episode/'policy_decisions').glob('*.npz')):
        with np.load(file, allow_pickle=False) as d:
            elapsed = float(d['elapsed'])
            meta = next(r for r in memory if abs(r['elapsed_seconds']-elapsed) < 1e-6)
            diag = next(r for r in diagnostics if abs(r['elapsed_seconds']-elapsed) < 1e-6)
            if meta['marker_override']:
                rows.append(dict(file=str(file), sha256=digest(file), elapsed_seconds=elapsed,
                    original_generation_and_ranking_reproduced=False,
                    integration_skipped='Recorded marker override is not ordinary exploration ranking', checks=[]))
                continue
            args = (d['data'], float(d['resolution']), d['origin'], d['robot'],
                    float(d['heading']), (), d['viewed'])
            kwargs = dict(inspection_focal=meta['inspection_focal_px'])
            raw_old, _ = edge_policy.options(*args, **kwargs)
            if len(raw_old) != diag['raw_candidates']:
                raise AssertionError('Original raw candidate count not reproduced')
            raw_matrix = matrix(raw_old)
            recorded = d['candidates']
            # Memory adjusts the revisit feature and removes candidates. It
            # must not disguise a different source geometry or gain function.
            for row in recorded:
                if not any(np.allclose(row[:8], r[:8], atol=1e-7, rtol=0) for r in raw_matrix):
                    raise AssertionError('Original eligible geometry/features not reproduced')
            old_options = [dict(index=i, features=row[4:]) for i, row in enumerate(recorded)]
            chosen = edge_policy.choose(old_options, d['weights'])
            if chosen is None or chosen['index'] != int(d['selected_index']):
                raise AssertionError('Original recorded ranking not reproduced')
            try:
                edge_policy.start_options = revised.options
                options, _ = edge_policy.options(*args, **kwargs)
            finally:
                edge_policy.start_options = original_start
            row = dict(file=str(file), sha256=digest(file), elapsed_seconds=elapsed,
                original_raw_candidates=len(raw_old), original_eligible_candidates=len(recorded),
                original_generation_and_ranking_reproduced=True,
                recorded_failed_viewpoints=meta['failed_viewpoints'],
                recorded_marker_override=meta['marker_override'],
                candidate_count=len(options),
                candidate_generation_unchanged=(matrix(options).shape == raw_matrix.shape and
                    bool(np.allclose(matrix(options), raw_matrix, atol=1e-7, rtol=0))),
                candidate_pocket_continuation=bool(options) and all(c.get('pocket_continuation') for c in options),
                candidate_all_transit=bool(options) and all(c.get('start_escape') for c in options),
                checks=[])
            if meta['failed_viewpoints'] or not row['candidate_all_transit']:
                row['integration_skipped'] = ('Failure cooldown needs unrecorded state' if meta['failed_viewpoints']
                    else 'Ordinary viewpoints should retain their existing memory suppression')
                rows.append(row)
                continue
            for transform in [(0., 0., 0.), (2.7, -1.8, 1.2), (-3., 4., -2.9)]:
                for prior_views in [False, True]:
                    state = ExplorationMemory()
                    for c in options:
                        x, y, yaw = transform_pose(c['x'], c['y'], c['yaw'], transform)
                        if prior_views:
                            state.views.append(dict(x=x, y=y, yaw=yaw,
                                gain=float(c['features'][1]), stamp=elapsed-1.))
                            # Many prior entries around every proposed goal
                            # exercise the bounded revisit penalty as well.
                            cx, cy = math.floor(x/.6), math.floor(y/.6)
                            for dy in range(-2, 3):
                                for dx in range(-2, 3):
                                    state.cells[(cx+dx, cy+dy)] = dict(entries=3, last_exit=elapsed-2.)
                    kept = state.filter(options, transform, elapsed)
                    selected = edge_policy.choose(kept, d['weights'])
                    if len(kept) != len(options) or selected is None:
                        raise AssertionError('Transit was erased by ordinary view memory')
                    state.selected(selected, transform)
                    state.completed(True, elapsed+.5)
                    if state.pending is not None or len(state.views) != (len(options) if prior_views else 0):
                        raise AssertionError('Transit completion incorrectly stored a viewed surface')
                    row['checks'].append(dict(transform=list(transform),duplicate_views_injected=prior_views,
                        kept=len(kept),selected_xy=[selected['x'], selected['y']],
                        selected_transit=bool(selected.get('start_escape')),surface_view_credit_added=False))
            row['selection_same_in_all_stress_cases'] = all(
                np.allclose(c['selected_xy'], row['checks'][0]['selected_xy'], atol=1e-9, rtol=0)
                for c in row['checks'])
            rows.append(row)
    sources = [episode/'planning_diagnostics.jsonl', episode/'exploration_memory.jsonl',
               old/'edge_policy.py', old/'exploration_memory.py', old/'surface_endpoint_policy.py',
               old/'start_waypoints.py', new/'start_waypoints.py']
    return dict(evidence_type='Recorded planner-generation/ranking replay plus explicit counterfactual memory stress',
        rows=rows, decisions=len(rows), original_generation_and_ranking_checks=sum(
            r['original_generation_and_ranking_reproduced'] for r in rows),
        counterfactual_filter_selection_checks=sum(len(r['checks']) for r in rows),
        policy_ground_truth_input=False, native_success_established=False,
        source_sha256={str(p): digest(p) for p in sources},
        limits=['The complete original odom memory and action timing are not archived',
                'Injected duplicate-view/revisit states test an interface; they are not the native memory',
                'A selected safe map waypoint does not establish Nav2 execution or departure success',
                'No future map, marker recognition, path, collision or duration is predicted'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['episode', 'baseline', 'candidate', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve the original integration audit')
    result = audit(args.episode.resolve(), args.baseline.resolve(), args.candidate.resolve())
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['decisions', 'original_generation_and_ranking_checks',
                                          'counterfactual_filter_selection_checks']}))
