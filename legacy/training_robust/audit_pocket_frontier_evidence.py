"""Replay observed planner maps to isolate frontier-source gain leakage.

This does not roll out future maps or reconstruct unrecorded exploration memory.
It never loads evaluator geometry, target positions or robot ground truth.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from audit_pocket_selection import matrix


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(episode, baseline, candidate):
    old = baseline/'src/osp_mission/osp_mission'
    new = candidate/'src/osp_mission/osp_mission'
    hashes = [{p.name: digest(p) for p in src.glob('*.py')} for src in [old, new]]
    if {k for k in hashes[0].keys() | hashes[1].keys()
            if hashes[0].get(k) != hashes[1].get(k)} != {'start_waypoints.py'}:
        raise ValueError('Require an isolated frontier-evidence change')
    sys.path.insert(0, str(old.parent))
    from osp_mission import edge_policy
    if Path(edge_policy.__file__).resolve() != (old/'edge_policy.py').resolve():
        raise ValueError('Unexpected planner import')
    spec = importlib.util.spec_from_file_location('pocket_frontier_candidate', new/'start_waypoints.py')
    revised = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(revised)
    original_start = edge_policy.start_options
    memories = [json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
    diagnostics = [json.loads(line) for line in (episode/'planning_diagnostics.jsonl').read_text().splitlines()]
    rows = []
    for file in sorted((episode/'policy_decisions').glob('*.npz')):
        with np.load(file, allow_pickle=False) as d:
            elapsed = float(d['elapsed'])
            memory = next(r for r in memories if abs(r['elapsed_seconds']-elapsed) < 1e-6)
            if memory['marker_override']:
                rows.append(dict(file=str(file), sha256=digest(file), elapsed_seconds=elapsed,
                                 skipped='Marker override is outside this comparison'))
                continue
            diag = next(r for r in diagnostics if abs(r['elapsed_seconds']-elapsed) < 1e-6)
            args = (d['data'], float(d['resolution']), d['origin'], d['robot'],
                    float(d['heading']), (), d['viewed'])
            kwargs = dict(inspection_focal=memory['inspection_focal_px'])
            choices, _ = edge_policy.options(*args, **kwargs)
            before = matrix(choices)
            if len(choices) != diag['raw_candidates']:
                raise AssertionError('Original raw candidate count differs')
            for row in d['candidates']:
                if not any(np.allclose(row[:8], raw[:8], atol=1e-7, rtol=0) for raw in before):
                    raise AssertionError('Recorded eligible geometry/features differ')
            ranked = [dict(index=i, features=row[4:]) for i, row in enumerate(d['candidates'])]
            if edge_policy.choose(ranked, d['weights'])['index'] != int(d['selected_index']):
                raise AssertionError('Recorded original selection differs')
            try:
                edge_policy.start_options = revised.options
                altered, _ = edge_policy.options(*args, **kwargs)
            finally:
                edge_policy.start_options = original_start
            after = matrix(altered)
            res = float(d['resolution'])
            regions, _ = label((d['data'] == 0) & (distance_transform_edt(d['data'] <= 50)*res >= .32))
            x, y = np.floor((d['robot']-d['origin'])/res).astype(int)
            region = regions[y, x]
            area = float(np.count_nonzero(regions == region)*res**2) if region else 0.
            rows.append(dict(file=str(file), sha256=digest(file), elapsed_seconds=elapsed,
                original_generation_and_selection_reproduced=True, normal_component_area_m2=area,
                original_count=len(choices), candidate_count=len(altered),
                original_all_transit=bool(choices) and all(c.get('start_escape', False) for c in choices),
                candidate_all_transit=bool(altered) and all(c.get('start_escape', False) for c in altered),
                raw_candidates_identical=before.shape == after.shape and bool(np.allclose(before, after, atol=1e-7, rtol=0)),
                recorded_selected_xy=d['candidates'][int(d['selected_index']), :2].tolist(),
                failed_viewpoints=memory['failed_viewpoints']))
    return dict(evidence_type='Observed map candidate-generation replay; not a new native trajectory',
        rows=rows, checked_decisions=sum(r.get('original_generation_and_selection_reproduced', False) for r in rows),
        source_sha256={str(p): digest(p) for p in [old/'start_waypoints.py', new/'start_waypoints.py',
            old/'edge_policy.py', old/'frontier.py', episode/'exploration_memory.jsonl', episode/'planning_diagnostics.jsonl']},
        ground_truth_used=False, native_success_established=False, production_promoted=False,
        limits=['Candidate future maps, motion, memory, collisions and mission outcome are not simulated',
                'Changed raw normal viewpoints still need the native memory filter and action execution'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['episode', 'baseline', 'candidate', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve previous evidence')
    report = audit(a.episode.resolve(), a.baseline.resolve(), a.candidate.resolve())
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(checked_decisions=report['checked_decisions'],
        changed=sum(not r.get('raw_candidates_identical', True) for r in report['rows']),
        transit_to_normal=sum(r.get('original_all_transit') and not r.get('candidate_all_transit')
            for r in report['rows'] if not r.get('skipped')))))
