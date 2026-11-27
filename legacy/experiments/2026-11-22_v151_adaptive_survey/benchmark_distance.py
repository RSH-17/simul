"""Compare exact reachable grids and timings on saved policy inputs without ROS."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from statistics import median
import sys
import time

import numpy as np


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.Grid


def run(parent, workspace, decisions):
    sys.path.insert(0, str(workspace / 'src/osp_mission'))
    rel = Path('src/osp_mission/osp_mission/_r32_navigation_core.py')
    versions = [load(parent/rel, 'frozen_reachability'), load(workspace/rel, 'new_reachability')]
    rows = []
    files = sorted(decisions.glob('*.npz'))
    if not files:
        raise ValueError('No saved planning decisions')
    for i in np.unique(np.linspace(0, len(files)-1, min(len(files), 12), dtype=int)):
        f = files[i]
        with np.load(f) as sample:
            values = [(), ()]
            elapsed = [[], []]
            for repeat in range(4):
                for j in ((0, 1) if repeat % 2 == 0 else (1, 0)):
                    grid = versions[j](sample['data'], float(sample['resolution']), (*sample['origin'], 0.))
                    pose = sample['robot']
                    begin = time.perf_counter()
                    values[j] = grid.reachable(pose, .32)
                    elapsed[j].append(1000 * (time.perf_counter()-begin))
            for original, new in zip(*values):
                np.testing.assert_array_equal(original, new)
            rows.append(dict(file=f.name, sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
                             shape=list(grid.cells.shape), reachable=int(np.isfinite(values[0][1]).sum()),
                             baseline_ms=median(elapsed[0]), candidate_ms=median(elapsed[1])))
    return dict(environment='offline Mac CPU, four alternating calls per saved input',
                saved_run=decisions.parent.parent.name, exact_equal=True, rows=rows,
                median_baseline_ms=median(r['baseline_ms'] for r in rows),
                median_candidate_ms=median(r['candidate_ms'] for r in rows))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('parent','workspace','decisions','output'):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    result = run(args.parent, args.workspace, args.decisions)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(result['median_baseline_ms'], result['median_candidate_ms'])
