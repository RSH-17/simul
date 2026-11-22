"""Validate printed marker visibility on seeded SDF fixtures, without driving a policy."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from fixture import CALIBRATION, build, read
from witness_audit import inspect


def probe(anchor, namespace, layouts, official_exam=False):
    calibration = read(CALIBRATION)
    seeds, ids, print_count, visibility = set(), set(), Counter(), Counter()
    failures = []
    objects = []
    host_mix = Counter()
    table_mode = 'fixed' if official_exam else 'shuffle'
    for i in range(layouts):
        seed = int.from_bytes(hashlib.sha256(f'{namespace}/{i}'.encode()).digest()[:4], 'big')
        if seed in seeds:
            raise ValueError('Repeated seed in fixture check')
        seeds.add(seed)
        count = 3 if official_exam else int(np.random.default_rng([seed, 0x544234]).integers(1, 6))
        with TemporaryDirectory() as temp:
            fixture = Path(temp)/'fixture'
            manifest = build(anchor, fixture, seed, count, table_mode)
            targets = inspect(fixture, manifest, calibration)['targets']
            assert len(targets) == count and len(set(manifest['ids'])) == count
            objects.append(manifest['movable_objects'])
            ids.update(manifest['ids'])
            print_count[count] += 1
            wall_count = sum(t['host'].startswith('wall_') for t in targets)
            host_mix[f'{wall_count}_wall_{count - wall_count}_object'] += 1
            for target in targets:
                complete = target['same_connected_grid_region'] and target['optical_witness_18px']
                visibility['checked'] += 1
                visibility['visible_and_grid_connected'] += int(complete)
                if not complete:
                    failures.append(dict(seed=seed, **target))
    return dict(namespace=namespace, layouts=layouts, tables=table_mode, count_distribution=print_count,
                layouts_by_attachment=host_mix, layouts_by_added_objects=Counter(objects),
                distinct_ids=len(ids), movable_object_range=[min(objects),max(objects)],
                visibility=visibility, failures=failures)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('anchor',type=Path)
    cli.add_argument('output',type=Path)
    cli.add_argument('--namespace',default='turtlebot4-8780-fixture-followup-v1')
    cli.add_argument('--layouts',type=int,default=192)
    cli.add_argument('--official-exam', action='store_true',
                     help='Keep the original desks and place exactly three independently drawn prints.')
    args=cli.parse_args()
    if args.layouts<1:cli.error('--layouts must be positive')
    result=probe(args.anchor,args.namespace,args.layouts,args.official_exam)
    args.output.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    print(result['visibility'])
    if result['failures']:raise SystemExit(1)
