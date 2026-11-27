"""Explain frozen goal scores using recorded policy observations only.

Sensitivity re-ranks the SAME candidate list and memory state. It predicts
neither future maps, collision safety, task success nor an improved trajectory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


COLUMNS = ['x', 'y', 'yaw', 'scan_angle', 'surface_gain', 'frontier_gain',
           'distance', 'turn', 'revisit']


def components(features, weights):
    denominator = weights[2]*features[:, 2]/.26 + weights[3]*features[:, 3]/.5 + 5.
    surface = weights[0]*features[:, 0]/denominator
    frontier = weights[1]*features[:, 1]/denominator
    revisit = weights[4]*features[:, 4]*.1
    return surface, frontier, revisit, surface+frontier-revisit


def audit(inputs, workspace, frontier_factors=(.125, .25, .5, 1., 2.), deadline_seconds=None):
    sys.path.insert(0, str(workspace/'src/osp_mission'))
    from osp_mission.surface_endpoint_policy import choose
    from osp_mission.time_budget import choose as before_deadline
    if deadline_seconds is not None and (not np.isfinite(deadline_seconds) or deadline_seconds<=0):
        raise ValueError('Deadline must be a finite positive mission duration')
    source = Path(sys.modules[choose.__module__].__file__).resolve()
    expected_source = (workspace/'src/osp_mission/osp_mission/surface_endpoint_policy.py').resolve()
    if source != expected_source:
        raise ValueError('Imported chooser is not from the requested frozen workspace')
    deadline_source=Path(sys.modules[before_deadline.__module__].__file__).resolve()
    if deadline_source!=(workspace/'src/osp_mission/osp_mission/time_budget.py').resolve():
        raise ValueError('Imported deadline chooser is not from the requested frozen workspace')
    rows = []
    for file in sorted(inputs.glob('*.npz')):
        with np.load(file, allow_pickle=False) as d:
            if d['candidate_columns'].tolist() != COLUMNS:
                raise ValueError(f'Unsupported audit schema: {file}')
            candidates, weights = d['candidates'], d['weights']
            if candidates.ndim != 2 or candidates.shape[1] != 9 or weights.shape != (5,):
                raise ValueError(f'Invalid audit shape: {file}')
            if not np.isfinite(candidates).all() or not np.isfinite(weights).all() or np.any(weights <= 0):
                raise ValueError(f'Nonfinite or invalid audit inputs: {file}')
            selected = int(d['selected_index'])
            if not -1 <= selected < len(candidates):
                raise ValueError(f'Invalid selected index: {file}')
            row = dict(file=str(file), sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                       elapsed_s=float(d['elapsed']), candidate_count=len(candidates),
                       recorded_selected_index=selected, weights=weights.tolist(), robot=d['robot'].tolist())
            if not len(candidates):
                row['empty_candidates'] = True
                rows.append(row)
                continue
            options = [dict(x=c[0], y=c[1], yaw=c[2], scan_angle=c[3], features=c[4:], index=i)
                       for i, c in enumerate(candidates)]
            features = candidates[:, 4:]

            def explain(index, w):
                surface, frontier, revisit, scores = components(features, w)
                c = candidates[index]
                return dict(index=int(index), xy=c[:2].tolist(), features=c[4:].tolist(),
                            planned_path_m=float(c[6]), surface_score=float(surface[index]),
                            frontier_score=float(frontier[index]), revisit_penalty=float(revisit[index]),
                            score=float(scores[index]))

            def checked_choice(w):
                result = choose(options, w)['index']
                if result != int(np.argmax(components(features, w)[-1])):
                    raise AssertionError(f'Score decomposition diverged from frozen chooser: {file}')
                return result

            baseline = checked_choice(weights)
            row.update(score_selected_index=baseline, recorded_selection_reproduced=selected == baseline,
                       baseline=explain(baseline, weights))
            if deadline_seconds is not None:
                remaining=deadline_seconds-row['elapsed_s']
                chosen=before_deadline(options,weights,remaining)
                original=options[baseline]
                required=(original['features'][2]/.20+original['features'][3]/.45
                          +abs(original['scan_angle'])/.45+3.5)
                row['deadline']=dict(remaining_s=remaining,original_nominal_required_s=float(required),
                    original_exceeds_nominal_budget=bool(required>remaining),
                    alternative=None if chosen is None else explain(chosen['index'],weights),
                    original_candidate_identity_preserved=chosen is None or any(chosen is c for c in options),
                    interpretable_as_normal_selection=selected==baseline,
                    score_note='Alternative score shown at original weights, before any deadline discount')
            # Marker follow-up/start overrides cannot be identified from this
            # schema. Preserve mismatches; never call them score decisions.
            nearby = np.flatnonzero((features[:, 0] > 0) & (features[:, 2] <= 2.))
            if len(nearby):
                index = nearby[np.argmax(components(features, weights)[-1][nearby])]
                row['best_nearby_surface_candidate'] = explain(index, weights)
            row['sensitivity'] = []
            for factor in frontier_factors:
                w = weights.copy()
                w[1] *= factor
                choice = checked_choice(w)
                row['sensitivity'].append(dict(frontier_weight_factor=factor,
                                               **explain(choice, w)))
            rows.append(row)
    matched = [r for r in rows if r.get('recorded_selection_reproduced')]
    return dict(evidence_type='Offline ranking audit of recorded native observations; no new trajectory',
                workspace=str(workspace), inputs=str(inputs), decisions=len(rows),
                recorded_selections_reproduced=len(matched),
                chooser_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), rows=rows,
                deadline_seconds=deadline_seconds,
                deadline_chooser_sha256=hashlib.sha256(deadline_source.read_bytes()).hexdigest(),
                weights_retrained=False, production_promoted=False, ground_truth_used=False,
                limitations=['Long goals or revisits can be necessary; distance alone is not failure',
                             'Sensitivity preserves the recorded candidate list, map and memory',
                             'No prediction of future success, safety, duration or traveled path',
                             'Low frontier weight can waste time inspecting ordinary walls'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs', 'workspace', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--deadline-seconds',type=float)
    args = parser.parse_args()
    result = audit(args.inputs.resolve(), args.workspace.resolve(),deadline_seconds=args.deadline_seconds)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}))
