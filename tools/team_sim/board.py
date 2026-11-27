"""Local, read-only experiment dashboard for each teammate's own results."""
import argparse
import csv
import json
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
    except (OSError,ValueError):elapsed=0;ids=[]
    protocol=read(folder/'protocol.json',{})
    randomized=protocol.get('fixture_mode')=='randomized_exam'
    exam=read(folder/'exam-score.json',{}) if randomized else {}
    if randomized:
        approached=sorted(int(i) for i,r in exam.get('records',{}).items() if r['verified_s'] is not None)
    else:
        approached=sorted(map(int,metric.get('registered_and_within_s',{})))
    return dict(id=folder.name,scene=scene,path=points,ids=metric.get('final_ids',ids),elapsed=metric.get('elapsed_sim_s',elapsed),
        approached=approached,reason=metric.get('reason','주행 중'),
        strict=exam.get('strict') if randomized else metric.get('strict_success'),bumps=metric.get('bumper_events'),
        finished=bool(exam) if randomized else (folder/'audit.json').exists(),
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
