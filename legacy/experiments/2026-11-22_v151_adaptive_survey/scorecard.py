"""Persist a paired scorecard as native runs finish, including a durable final status."""
import argparse
import json
from pathlib import Path
import time
import tarfile

from compare import report
from board_audit import reconcile


def save(path,data):
    tmp=path.with_suffix('.new')
    tmp.write_text(json.dumps(data,indent=2)+'\n')
    tmp.replace(path)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    p.add_argument('--deadline-hours',type=float,default=8.)
    p.add_argument('--board',help='Read-only comparison HTTP endpoint on the VM')
    p.add_argument('--cards',type=Path)
    p.add_argument('--export',type=Path,help='On completion, export metrics and verified synthetic videos here')
    args=p.parse_args()
    root=args.root.resolve()
    end=time.monotonic()+args.deadline_hours*3600.
    last=None
    board_done=set()
    while time.monotonic()<end:
        changed=tuple((root/k/'report.json').stat().st_mtime_ns if (root/k/'report.json').exists() else None
                      for k in ('baseline','candidate'))
        if changed!=last:
            if args.board and args.cards:
                for role in ('baseline','candidate'):
                    saved=root/role/'runs.json'
                    if not saved.is_file():
                        continue
                    for name in json.loads(saved.read_text()):
                        if name in board_done:
                            continue
                        try:
                            status=reconcile(Path(name),args.cards,args.board)
                            if status in ('current','cache refreshed'):
                                board_done.add(name)
                            print('BOARD',Path(name).name,status,flush=True)
                        except (OSError,ValueError,RuntimeError) as exc:
                            print('BOARD unavailable',name,str(exc),flush=True)
            data=report(root)
            save(root/'paired_comparison.json',data)
            count=sum(len(data[group]['pairs']) for group in ('regression','heldout'))
            save(root/'scorecard_status.json',dict(final=data['complete'],paired=count,updated_unix=time.time()))
            print('PAIRS',count,'COMPLETE',data['complete'],flush=True)
            if data['complete']:
                if args.export:
                    from export import export
                    from plot import plot
                    out=args.export.resolve()
                    export(root,out)
                    for split in ('regression','heldout'):
                        plot(out,out/(split+'-routes.png'),split)
                    archive=out.with_suffix('.tgz')
                    with tarfile.open(archive,'x:gz') as tgz:
                        tgz.add(out,arcname=out.name)
                    print('EXPORT',archive,flush=True)
                break
            last=changed
        time.sleep(30)
    else:
        raise TimeoutError('Deadline reached before a complete native suite; partial report retained')
