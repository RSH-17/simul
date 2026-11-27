"""Local, read-only experiment dashboard for each teammate's own results."""
import argparse
import csv
import json
import math
import os
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import parse_qs,urlsplit

HERE=Path(__file__).resolve().parent
ROOT=Path(os.environ.get('TEAM_SIM_HOME',str(Path.home()/'.local/share/simul-team'))).resolve()/'results'

def read(p,fallback):
    try:return json.loads(p.read_text())
    except (OSError,ValueError):return fallback

def jsonl(path):
    try:
        with path.open() as f:lines=f.readlines()
    except OSError:return []
    rows=[]
    for line in lines:
        try:rows.append(json.loads(line))
        except ValueError:pass  # the last line may still be half written
    return rows

def detections(folder):
    """First sighting of every marker ID and where that time comes from.

    Runs with the judge's detections.jsonl: every detector reading, live, in
    seconds since the mission start. Older runs fall back to the policy's own
    logs (follow-ups and directed looks live, graph observations after the
    run) on the policy clock; there a marker registered on its first sighting
    appears only after the run.
    """
    judged=jsonl(folder/'detections.jsonl')
    if judged:
        # Judge log: every detector reading, seconds since the mission start.
        seen={}
        for row in judged:
            ident,t=row.get('id'),row.get('t')
            if isinstance(ident,int) and isinstance(t,(int,float)):
                seen[str(ident)]=min(t,seen.get(str(ident),t))
        return {k:round(v,1) for k,v in sorted(seen.items(),key=lambda kv:kv[1])},'judge'
    rows=jsonl(folder/'native/marker_followups.jsonl')+jsonl(folder/'native/marker_directed_looks.jsonl')
    for path in folder.glob('native/*/graph_marker_observations.json'):
        found=read(path,[])
        rows+=found if isinstance(found,list) else []
    seen={}
    for row in rows:
        ident,stamp=row.get('id'),row.get('stamp')
        if isinstance(ident,int) and isinstance(stamp,(int,float)):
            seen[str(ident)]=min(stamp,seen.get(str(ident),stamp))
    return {k:round(v,1) for k,v in sorted(seen.items(),key=lambda kv:kv[1])},'policy'

def live_verdict(folder,scene,trace):
    """Provisional evaluator-side verdict while a run is still going.

    Mirrors the judge: an approach counts once the ID is registered and the
    true pose is within 1.5 m; bumps are the judge's bumper events; the map
    error places the latest accepted detection in the world with the start
    pose (map frame = robot pose at the mission start). The policy's own
    three-frame sensor proof lives only in the judge, so the final strict
    verdict still comes from metrics.json.
    """
    truth={t['id']:(t['x'],t['y']) for t in scene.get('targets',[])}
    records={i:dict(registered_s=None,near_s=None,closest_m=None,map_error_m=None) for i in truth}
    registered=set()
    for row in trace:
        try:t,x,y=float(row['elapsed_sim_s']),float(row['world_x']),float(row['world_y'])
        except (KeyError,TypeError,ValueError):continue
        current={int(i) for i in (row.get('registered_ids') or '').split()}
        registered|=current
        for i,(tx,ty) in truth.items():
            r=records[i];d=math.hypot(x-tx,y-ty)
            if i in current and r['registered_s'] is None:r['registered_s']=round(t,1)
            if r['closest_m'] is None or d<r['closest_m']:r['closest_m']=round(d,2)
            if i in current and d<=1.5 and r['near_s'] is None:r['near_s']=round(t,1)
    try:
        with (folder/'headings.csv').open() as f:first=next(csv.DictReader(f))
        ox,oy,yaw=float(first['x']),float(first['y']),float(first['yaw_rad'])
        c,s=math.cos(yaw),math.sin(yaw)
        for row in jsonl(folder/'detections.jsonl'):
            i,p=row.get('id'),row.get('map_point')
            if row.get('kind')=='accepted' and i in truth and p:
                wx,wy=c*p[0]-s*p[1]+ox,s*p[0]+c*p[1]+oy
                records[i]['map_error_m']=round(math.hypot(wx-truth[i][0],wy-truth[i][1]),3)
    except (OSError,StopIteration,KeyError,TypeError,ValueError):pass
    bumps=sum(1 for e in jsonl(folder/'events.jsonl') if e.get('event')=='bumper')
    wrong=sorted(registered-set(truth))
    met=bool(truth) and all(r['near_s'] is not None and r['map_error_m'] is not None and r['map_error_m']<=.25 for r in records.values()) and not bumps and not wrong
    return dict(targets={str(i):r for i,r in records.items()},bumps=bumps,wrong_ids=wrong,conditions_met=met)

def detail(folder):
    scene=read(folder/'scene.json',{})
    scene['partitions']=scene.get('partitions',[])+read(folder/'run/simulation/demo_layout.json',{}).get('tables',[])
    try:
        with (folder/'headings.csv').open() as f:rows=list(csv.DictReader(f))
        points=[[float(r[k]) for k in ('x','y')] for r in rows if all(r.get(k) for k in ('x','y'))]
        points=points[::max(1,len(points)//2000)]
    except (OSError,ValueError):points=[]
    metric=read(folder/'metrics.json',{})
    try:
        with (folder/'trajectory.csv').open() as f:trace=list(csv.DictReader(f))
        last=next((r for r in reversed(trace) if r.get('registered_ids') is not None and r.get('elapsed_sim_s')), {})
        elapsed=float(last.get('elapsed_sim_s',0));ids=last.get('registered_ids','').split()
    except (OSError,ValueError):trace=[];elapsed=0;ids=[]
    protocol=read(folder/'protocol.json',{})
    randomized=protocol.get('fixture_mode')=='randomized_exam'
    exam=read(folder/'exam-score.json',{}) if randomized else {}
    if randomized:
        approached=sorted(int(i) for i,r in exam.get('records',{}).items() if r['verified_s'] is not None)
    else:
        approached=sorted(map(int,metric.get('registered_and_within_s',{})))
    detected,detected_source=detections(folder)
    finished=bool(exam) if randomized else (folder/'audit.json').exists()
    live=None if finished or randomized else live_verdict(folder,scene,trace)
    return dict(id=folder.name,scene=scene,path=points,ids=metric.get('final_ids',ids),elapsed=metric.get('elapsed_sim_s',elapsed),
        approached=approached,detected=detected,detected_source=detected_source,reason=metric.get('reason','주행 중'),
        strict=exam.get('strict') if randomized else metric.get('strict_success'),bumps=metric.get('bumper_events'),
        finished=finished,live=live,
        interrupted=(folder/'interruption.json').exists(),protocol=protocol)

# A finished run (audit.json or interruption.json written) no longer changes.
CACHE={};LOCK=threading.Lock()

def runs():
    """Run folder names, newest first. Reading every run's CSVs here made /api/runs
    take ~17 s for 100 runs while the page polls every 2 s."""
    folders=[p for p in ROOT.glob('team-*-track-*') if p.is_dir()]
    return [p.name for p in sorted(folders,key=lambda p:p.stat().st_mtime,reverse=True)]

def cached_detail(folder):
    with LOCK:
        if folder.name in CACHE:return CACHE[folder.name]
    data=detail(folder)
    if (folder/'audit.json').exists() or (folder/'interruption.json').exists():
        with LOCK:CACHE[folder.name]=data
    return data

class Handler(BaseHTTPRequestHandler):
    def send(self,data,kind='application/json'):
        if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False).encode()
        try:
            self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass  # the browser left before the reply ended
    def do_GET(self):
        q=urlsplit(self.path);items=parse_qs(q.query)
        if q.path=='/':return self.send((HERE/'board.html').read_bytes(),'text/html; charset=utf-8')
        if q.path=='/api/runs':return self.send(runs())
        name=items.get('id',[''])[0];folder=(ROOT/name).resolve()
        if not name or folder.parent!=ROOT or not folder.is_dir():return self.send_error(404)
        if q.path=='/api/run':return self.send(cached_detail(folder))
        if q.path=='/camera.jpg':
            photos=sorted((folder/'camera').glob('*.jpg'))
            if photos:return self.send(photos[-1].read_bytes(),'image/jpeg')
        if q.path=='/video.mp4' and (folder/'audit.json').exists() and (folder/'mission.mp4').exists():
            return self.send((folder/'mission.mp4').read_bytes(),'video/mp4')
        if q.path=='/evaluation.json':
            exam=folder/'exam-score.json'
            return self.send(read(exam if exam.is_file() else folder/'metrics.json',{}))
        return self.send_error(404)
    def log_message(self,*args):pass

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8780);a=p.parse_args()
    ROOT.mkdir(parents=True,exist_ok=True)
    print(f'Team simulation board http://127.0.0.1:{a.port}/   {ROOT}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
