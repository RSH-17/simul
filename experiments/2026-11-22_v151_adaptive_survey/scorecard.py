"""Persist a paired scorecard as native runs finish, including a durable final status."""
import argparse
import json
from pathlib import Path
import time

from compare import report


def save(path,data):
    tmp=path.with_suffix('.new')
    tmp.write_text(json.dumps(data,indent=2)+'\n')
    tmp.replace(path)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    p.add_argument('--deadline-hours',type=float,default=8.)
    args=p.parse_args()
    root=args.root.resolve()
    end=time.monotonic()+args.deadline_hours*3600.
    last=None
    while time.monotonic()<end:
        changed=tuple((root/k/'report.json').stat().st_mtime_ns if (root/k/'report.json').exists() else None
                      for k in ('baseline','candidate'))
        if changed!=last:
            data=report(root)
            save(root/'paired_comparison.json',data)
            count=sum(len(data[group]['pairs']) for group in ('regression','heldout'))
            save(root/'scorecard_status.json',dict(final=data['complete'],paired=count,updated_unix=time.time()))
            print('PAIRS',count,'COMPLETE',data['complete'],flush=True)
            if data['complete']:
                break
            last=changed
        time.sleep(30)
    else:
        raise TimeoutError('Deadline reached before a complete native suite; partial report retained')
