"""Repeat a surprising 5/10-FPS result and check intermediate/stress settings."""
import fcntl,json,os,subprocess,sys
from pathlib import Path
from suite import native,HERE

root=Path(sys.argv[1]).resolve()
assert (root/'suite_complete.json').is_file(),'Finish initial validation first'
locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
for handle in locks:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
plan=[(112021,480,10,'repeat-a'),(112021,480,5,'repeat-a'),
      (112021,480,5,'repeat-b'),(112021,480,10,'repeat-b'),
      (112022,400,10,'intermediate'),(112022,480,15,'high-fps')]
(root/'followup_plan.json').write_text(json.dumps(dict(reason='Repeat non-monotonic 480/5 versus 480/10 closed-loop outcome; add middle and high-FPS settings',
    conditions=plan),indent=2)+'\n')
rows=json.loads((root/'runs.json').read_text())
for map_id,size,fps,tag in plan:
    run=native(root,map_id,size,fps,tag,90)
    rows.append(str(run));(root/'runs.json').write_text(json.dumps(rows,indent=2)+'\n')
subprocess.run([sys.executable,str(HERE/'summarize.py'),str(root)],check=True)
(root/'followup_complete.json').write_text(json.dumps(dict(completed=True,native_validation_runs=len(rows),extra_runs=len(plan)))+'\n')
