"""Local, read-only experiment dashboard for each teammate's own results."""
import argparse
import csv
import json
import math
import os
import struct
import threading
import zlib
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

REASONS={'unseen_reachable_object':'아직 안 본 닿을 수 있는 물체','midmission_wall_slot':'주행 중 벽 구간 확인',
    'persistent_small_wall_candidate':'계속 남은 작은 벽 후보','interior_first_exploration':'실내 먼저 탐색',
    'quick_perimeter_pass':'빠른 외곽 한 바퀴','explore_open_frontier':'열린 프런티어 탐색','finish_current_box':'보던 상자 마저 확인',
    'regional_camera_gain_per_route':'경로 대비 카메라 이득이 큰 구역','inspect_other_surface':'다른 면 확인',
    'verified_marker_approach':'확인된 마커 접근','discover_beyond_inspected_boxes':'확인한 상자 너머 탐색',
    'remaining_time_for_mapped_walls':'남은 시간으로 지도상 벽 확인','wall_marker_evidence':'벽 마커 단서',
    'accepted_marker_followup':'검출 마커 재확인','region_leave':'구역 떠남','region_resume':'구역 복귀',
    'no_untried_reachable_interior_view':'시도 안 한 실내 관측 지점 없음','safe_frontier_fallback':'안전한 프런티어로 대체',
    'white_leg_candidate':'흰 다리 후보','box_safe_retry':'상자 안전 재시도','local_print_candidate':'근처 인쇄물 후보'}
NPZ={};MAP_PNG={};NPZ_LOCK=threading.Lock()
RULE_ORDER={'fresh':'정렬: 끝난 상자 아닌 면 → 같은 상자 이어서 → 형태를 맞춘 면 → 걸어서 가까운 순',
    'interior':'정렬: 끝난 상자 아닌 면 → 같은 상자 이어서 → 형태를 맞춘 면 → 걸어서 가까운 순',
    'deferred':'끝난 상자의 면(다른 후보가 없을 때만)',
    'wall':'벽 점수(낮을수록 우선) = 경로 거리 + 서는 거리·옆 이동 벌점 + 시도 횟수×2 + 벽 이어짐 벌점',
    'frontiers':'프런티어 점수 = 이득 ÷ (1 + 0.18×이동거리 + 2 m 미만 벌점 + 재방문)'}
RULE_NAME={'fresh':'새 물체 면','interior':'물체 면','deferred':'끝난 상자 면','wall':'벽 관측 지점','frontiers':'프런티어'}
REASON_LIST={'unseen_reachable_object':'fresh','inspect_other_surface':'fresh','finish_current_box':'fresh',
    'explore_open_frontier':'frontiers','completed_box_fallback':'deferred'}

def rule_line(kind,rank,row):
    """One compared candidate as a short Korean phrase."""
    if kind in ('fresh','interior','deferred'):
        flags=[f for f,on in (('끝난 상자',row.get('completed_box')),('같은 상자 이어서',row.get('local_batch')),
                              ('형태 맞춘 면',row.get('fitted_face')),(f"시도 {row.get('previous_attempts')}회",row.get('previous_attempts'))) if on]
        return f"{rank}위 걸어서 {row.get('path_m')}m"+(' · '+', '.join(flags) if flags else '')
    if kind=='wall':return f"{rank}위 점수 {row.get('score')} (직선 {row.get('straight_m')}m"+(f", 시도 {row['previous_attempts']}회" if row.get('previous_attempts') else '')+')'
    return f"{rank}위 점수 {row.get('score')} (이득 {row.get('gain')}, 이동 {row.get('travel_m')}m)"

def rule_view(row,world):
    """What the phased hybrid compared for one goal (rule_trace.jsonl, v1.5.7.1+)."""
    trace=row.get('trace') if isinstance(row.get('trace'),dict) else None
    chosen=row.get('chosen_from')
    if trace is None:return dict(available=False,text='이 결정에는 규칙 비교 기록이 없음 (마커 접근 등 규칙 밖 결정)',points=[])
    if 'error' in trace:return dict(available=False,text='규칙 비교 기록 실패: '+str(trace['error']),points=[])
    kind=chosen['list'] if chosen else REASON_LIST.get(row.get('reason')) or ('wall' if row.get('phase')=='wall' else 'fresh')
    rows=trace.get(kind) or []
    lines=[rule_line(kind,i+1,r) for i,r in enumerate(rows[:4])]
    head=f"{RULE_NAME.get(kind,kind)} {len(rows)}개 비교" if rows else f"{RULE_NAME.get(kind,kind)} 후보 없음"
    head+=f" → {chosen['rank']}위 선택" if chosen else ' → 고른 목표가 이 목록에 없음'
    if row.get('reason')=='midmission_wall_slot':
        slot=trace.get('slot') or {};tour=(slot.get('tour') or {})
        head+=f" · 벽 구간 순회(계획 {tour.get('plans',0)}번째 이후, 직선거리로 가장 짧은 순회의 다음 목표)"
    points=[dict(rank=i+1,world=world(r['x'],r['y'])) for i,r in enumerate(rows[:5]) if r.get('x') is not None and r.get('y') is not None]
    return dict(available=True,list=kind,rank=chosen['rank'] if chosen else None,order=RULE_ORDER.get(kind,''),
                text=head+(' | '+' / '.join(lines) if lines else ''),points=points,
                remaining_s=trace.get('remaining_s'),counts=trace.get('counts'))

def start_pose(folder):
    """World pose at the mission start; the SLAM map frame starts there with zero yaw."""
    try:
        with (folder/'headings.csv').open() as f:first=next(csv.DictReader(f))
        return float(first['x']),float(first['y']),float(first['yaw_rad'])
    except (OSError,StopIteration,KeyError,ValueError):return None

def png(rgba):
    """Minimal RGBA PNG encoder (no Pillow needed)."""
    h,w,_=rgba.shape
    raw=b''.join(b'\x00'+rgba[r].tobytes() for r in range(h))
    chunk=lambda kind,data:struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,6,0,0,0))+chunk(b'IDAT',zlib.compress(raw,6))+chunk(b'IEND',b'')

def map_knowledge(data,viewed,resolution,origin,pose,room):
    """What the robot's own SLAM map knows at one decision.

    data: OccupancyGrid values (-1 unknown, 0..100). viewed: the policy's camera
    evidence per cell side (EdgeCoverage); it keeps only sides read from a
    readable distance and is cleared where the map changes.
    """
    import numpy as np
    known=data>=0;free=data==0;occupied=data>50  # the policy's boundary_faces thresholds
    surface=np.zeros((4,)+data.shape,dtype=bool)
    for side,(dy,dx) in enumerate(((-1,0),(0,1),(1,0),(0,-1))):
        neighbour=np.zeros_like(free)
        rows=slice(max(0,-dy),data.shape[0]-max(0,dy));cols=slice(max(0,-dx),data.shape[1]-max(0,dx))
        neighbour[rows,cols]=free[max(0,dy):data.shape[0]-max(0,-dy) or None,max(0,dx):data.shape[1]-max(0,-dx) or None]
        surface[side]=occupied&neighbour
    seen=surface&viewed if viewed.shape==surface.shape else np.zeros_like(surface)
    out=dict(known_m2=round(float(known.sum())*resolution**2,1),free_m2=round(float(free.sum())*resolution**2,1),
             occupied_cells=int(occupied.sum()),surface_sides=int(surface.sum()),viewed_sides=int(seen.sum()),
             viewed_m=round(float(seen.sum())*resolution,2),surface_m=round(float(surface.sum())*resolution,1),
             viewed_ratio=round(float(seen.sum())/max(1,int(surface.sum())),4))
    if pose and room:
        ox,oy,yaw=pose;c,s_=math.cos(yaw),math.sin(yaw)
        rr,cc=np.nonzero(known)
        mx=origin[0]+(cc+.5)*resolution;my=origin[1]+(rr+.5)*resolution
        wx=c*mx-s_*my+ox;wy=s_*mx+c*my+oy
        inside=(abs(wx)<=room[0]/2)&(abs(wy)<=room[1]/2)
        out['room_known_ratio']=round(float(inside.sum())*resolution**2/(room[0]*room[1]),4)
    rgba=np.zeros(data.shape+(4,),dtype=np.uint8)
    rgba[free]=(110,170,205,70);rgba[occupied]=(235,235,235,215);rgba[seen.any(axis=0)]=(70,235,150,255)
    return out,png(np.ascontiguousarray(rgba[::-1]))

def candidate_score(row,w):
    """The planners' candidate score (surface_endpoint_policy / directional_policy)."""
    surface,frontier,distance,turn,revisit=row[4:9]
    return (w[0]*surface+w[1]*frontier)/(w[2]*distance/.26+w[3]*turn/.5+5.)-w[4]*revisit*.1

def decision_snapshot(path,pose=None,room=None):
    """One saved planner decision (files are written once, atomically)."""
    key=(str(path),path.stat().st_mtime_ns)
    with NPZ_LOCK:
        if key in NPZ:return NPZ[key]
    import numpy as np
    with np.load(path) as z:
        rows=z['candidates'];w=z['weights'];chosen=int(z['selected_index'])
        scores=[candidate_score(r,w) for r in rows]
        order=sorted(range(len(rows)),key=lambda i:-scores[i])
        pick=lambda i:dict(index=i,x=float(rows[i][0]),y=float(rows[i][1]),score=round(float(scores[i]),3),
                           features=[round(float(v),3) for v in rows[i][4:9]])
        try:
            data=z['data'];resolution=float(z['resolution']);origin=[float(v) for v in z['origin']]
            knowledge,image=map_knowledge(data,z['viewed'],resolution,origin,pose,room)
            grid=dict(width=int(data.shape[1]),height=int(data.shape[0]),resolution=resolution,origin=origin)
        except Exception:knowledge=image=grid=None  # older snapshot without the map
        snap=dict(t=round(float(z['elapsed']),1),planner=str(z['planner']),candidates=len(rows),map=knowledge,grid=grid,
                  robot=[float(v) for v in z['robot']],selected=pick(chosen) if 0<=chosen<len(rows) else None,
                  top=[pick(i) for i in order[:3]],score_best=bool(order) and order[0]==chosen)
    with NPZ_LOCK:NPZ[key]=snap;MAP_PNG[key]=image
    return snap

def visited_faces(folder):
    """Mission times at which the hybrid sent the robot to a surface it had not been sent to before.

    Same face as the policy's attempt history: anchors within 0.5 m and view
    directions within ~45 deg (cos > 0.7). The direction is from the face to the
    goal; center_wall.jsonl and hybrid_goals.jsonl have one row per goal, in order.
    """
    cw=jsonl(folder/'native/center_wall.jsonl');hg=jsonl(folder/'native/hybrid_goals.jsonl')
    if len(cw)!=len(hg):return []
    faces=[];new=[]
    for row,goal in zip(cw,hg):
        s=row.get('surface')
        if not s or not isinstance(row.get('elapsed'),(int,float)):continue
        try:nx,ny=goal['x']-s[0],goal['y']-s[1]
        except (KeyError,TypeError,IndexError):continue
        n=math.hypot(nx,ny) or 1.;nx,ny=nx/n,ny/n
        if not any(k==row.get('kind') and math.dist(p,s)<.5 and nx*a+ny*b>.7 for p,a,b,k in faces):
            new.append(row['elapsed'])
        faces.append((s,nx,ny,row.get('kind')))
    return new

def decisions(folder):
    """Decision timeline: every planner choice, the layer that made it and why.

    The policy logs each choice to native/policy_decisions (all candidates,
    their features, the weights and the chosen one) and the phased hybrid
    logs its reason to center_wall.jsonl at the same mission-elapsed time.
    From v1.5.7.1 the hybrid also logs what it compared (rule_trace.jsonl).
    Map-frame points are placed in the world with the start pose so they can
    be drawn on the board.
    """
    pose=start_pose(folder);ox,oy,yaw=pose or (0.,0.,0.)
    room=read(folder/'scene.json',{}).get('room_size')
    c,s=math.cos(yaw),math.sin(yaw)
    world=lambda x,y:[round(c*x-s*y+ox,3),round(s*x+c*y+oy,3)]
    reasons=[r for r in jsonl(folder/'native/center_wall.jsonl') if isinstance(r.get('elapsed'),(int,float))]
    traced=[r for r in jsonl(folder/'native/rule_trace.jsonl') if isinstance(r.get('elapsed'),(int,float))]
    rows=[];faces=visited_faces(folder)
    for path in sorted((folder/'native/policy_decisions').glob('*.npz')):
        try:snap=decision_snapshot(path,pose,tuple(room) if room else None)
        except Exception:continue  # unreadable snapshot
        why=min(reasons,key=lambda r:abs(r['elapsed']-snap['t']),default=None)
        if why is not None and abs(why['elapsed']-snap['t'])>.5:why=None
        kind=why.get('kind') if why else None
        if kind and kind!='flow_or_marker':layer='상위 규칙'
        elif snap['score_best']:layer='점수식'
        else:layer='마커 접근·경로 필터'
        reason=why.get('reason') if why else None
        item=dict(snap,layer=layer,reason=reason,reason_text=REASONS.get(reason,reason),kind=kind,
                  marker_id=why.get('marker_id') if why else None,phase=why.get('phase') if why else None)
        item['robot']=world(*snap['robot'])
        for k in ('selected',):
            if item[k]:item[k]=dict(item[k],world=world(item[k]['x'],item[k]['y']))
        item['top']=[dict(t,world=world(t['x'],t['y'])) for t in snap['top']]
        trace=min(traced,key=lambda r:abs(r['elapsed']-snap['t']),default=None)
        if trace is not None and abs(trace['elapsed']-snap['t'])>1.:trace=None
        item['rule']=rule_view(trace,world) if trace is not None else None
        if item.get('map') is not None:item['map']=dict(item['map'],faces_visited=sum(t<=snap['t']+.5 for t in faces))
        item['file']=path.name;item['map_frame']=[ox,oy,yaw] if pose else None
        rows.append(item)
    return rows

def map_png(folder,name):
    path=folder/'native/policy_decisions'/name
    if path.parent.name!='policy_decisions' or not path.is_file():return None
    pose=start_pose(folder);room=read(folder/'scene.json',{}).get('room_size')
    snap=decision_snapshot(path,pose,tuple(room) if room else None)
    key=(str(path),path.stat().st_mtime_ns)
    with NPZ_LOCK:return MAP_PNG.get(key)

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
        if q.path=='/api/decisions':return self.send(decisions(folder))
        if q.path=='/api/map.png':
            name=items.get('file',[''])[0]
            image=map_png(folder,Path(name).name) if name.endswith('.npz') else None
            if image:return self.send(image,'image/png')
            return self.send_error(404)
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
