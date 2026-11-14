"""Offline rasterization ablation; never changes a native run or uses truth in policy."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.ndimage import distance_transform_edt


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(episode, exporter, output):
    from osp_mission.edge_policy import options
    output.mkdir(parents=True, exist_ok=False)
    inputs = sorted((episode / 'policy_decisions').glob('*.npz'))
    datasets = sorted(episode.glob('*/map.data'))
    if not inputs or len(datasets) != 1:
        raise ValueError('Require recorded policy input and one finalized SLAM dataset')
    recorded = np.load(inputs[-1])
    report = dict(evidence_type='Offline occupancy reconstruction from finalized sensor graph; not a new native run',
                  episode=str(episode), snapshot=str(inputs[-1]), snapshot_sha256=digest(inputs[-1]),
                  dataset_sha256=digest(datasets[0]), exporter_sha256=digest(exporter),
                  policy_receives_truth=False, production_promoted=False, resolutions=[])
    for resolution in [.05, .025, .02]:
        filename = output / f'grid_{resolution}.json'
        subprocess.run([str(exporter), str(datasets[0]), str(resolution), '2', '.1', str(filename)],
                       check=True, capture_output=True, text=True)
        grid = json.loads(filename.read_text())
        data = np.asarray(grid['data'], dtype=np.int8).reshape(grid['height'], grid['width'])
        robot = recorded['robot']
        x, y = np.floor((robot - grid['origin']) / resolution).astype(int)
        clearance = distance_transform_edt(np.pad(data == 0, 1))[1:-1, 1:-1] * resolution
        choices, _ = options(data, resolution, grid['origin'], robot, float(recorded['heading']),
                             inspection_focal=308.)
        same_frame = data.shape == recorded['data'].shape and np.allclose(grid['origin'], recorded['origin'], atol=1e-8)
        report['resolutions'].append(dict(resolution_m=resolution, scans=grid['scans'],
            start_clearance_m=float(clearance[y, x]), required_body_clearance_m=.189 + resolution / np.sqrt(2) + .01,
            candidates=len(choices), transit_candidates=sum(c.get('start_escape', False) for c in choices),
            same_dimensions_and_origin_as_recorded=bool(same_frame),
            cells_different_from_recorded=int(np.count_nonzero(data != recorded['data'])) if same_frame else None,
            goal_distances_m=[float(c['distance']) for c in choices]))
    report['limitations'] = ['Uses finalized retained scans and corrected graph poses, not every original scan.',
                             'No counterfactual movement, new camera detections, or completion credit is inferred.']
    (output / 'audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--exporter', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    audit(a.episode.resolve(), a.exporter.resolve(), a.output.resolve())
