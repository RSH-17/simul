"""Check the actual frozen runtime adapter and ground-truth scorer without Gazebo."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import validate


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('suite',type=Path);p.add_argument('completed_run',type=Path)
    args=p.parse_args()
    m=validate.read(args.suite/'manifest.json')
    validate.assert_inputs(args.suite,m)
    row=validate.read(args.completed_run/'metrics.json')
    result=validate.measure(args.completed_run,{'files':{'world/osp_studio.sdf':row['world_sha256']}})
    if not result['valid'] or not result['strict'] or result['bumps'] or len(result['after_registration_near_s'])!=3:
        raise RuntimeError('Known complete native baseline failed independent same-frame score')
    with patch.object(validate,'screen_frozen',return_value={'eligible':False,'reasons':['disabled-for-test']}),             patch.object(validate.runner,'guard',side_effect=AssertionError('unexpected native path')):
        validate.execute(SimpleNamespace(root=args.suite,action='run'))
    print(json.dumps(dict(worlds_verified=len(m['worlds']),pairs=len(m['pairs']),reference_run=args.completed_run.name,
                          registered=result['registered_ids'],near=result['after_registration_near_s'],
                          camera=result['camera_info'],strict=result['strict'],disabled_launches=0)))


if __name__=='__main__':
    main()
