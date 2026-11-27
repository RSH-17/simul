"""Passive, unmodified JPEG exposure replay; run with opt-in candidate on PYTHONPATH."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vision_replay import project_targets
from osp_mission.edge_watch import EdgeWatch
from osp_mission.marker_pair import decode, printed_pattern_matches
from osp_mission.tiny_named_relook import TinyNamedRelook, aligned_turn


def camera(h, geom, t):
    x,y,a = [np.interp(t,h['elapsed_sim_s'],v) for v in (h['x'],h['y'],h['unwrapped'])]
    dx,dy,_=geom['translation']
    qx,qy,qz,qw=geom['quaternion']
    return ((x+math.cos(a)*dx-math.sin(a)*dy,y+math.sin(a)*dx+math.cos(a)*dy),
            a+math.atan2(2*(qy*qz-qx*qw),2*(qx*qz+qy*qw)),(x,y,a))


def run(folder, geom, window=None):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    scene=json.loads((folder/'scene.json').read_text())
    metrics=json.loads((folder/'metrics.json').read_text())
    headings=np.genfromtxt(folder/'headings.csv',names=True,delimiter=',')
    headings={s:headings[s] for s in headings.dtype.names}
    headings['unwrapped']=np.unwrap(headings['yaw_rad'])
    registered={int(k):v for k,v in metrics['registered_s'].items() if v is not None}
    watch,tracker = EdgeWatch(),TinyNamedRelook()
    fx,cx=geom['k'][0],geom['k'][2]
    calls=[]; native=[]; old_ms=[]; new_ms=[]; subthreshold=[]
    frames=neg=anon=0; older=EdgeWatch()
    with (folder/'frames.csv').open() as stream:
        for row in csv.DictReader(stream):
            i,t=int(row['frame']),float(row['elapsed_sim_s'])
            if window and not window[0]<=t<=window[1]:continue
            img=cv2.imread(str(folder/'camera'/f'{i:05d}.jpg'),cv2.IMREAD_GRAYSCALE)
            if img is None:raise ValueError(f'{folder.name} missing frame {i}')
            frames+=1; h,w=img.shape
            origin,yaw,base=camera(headings,geom,t)
            boxes=project_targets(base,scene,geom)
            is_negative=all(b[2]<-64 or b[0]>w+64 for b in boxes.values())
            neg+=int(is_negative)
            small=[]; tiny=[]
            if window:
                quads,raw,_=cv2.aruco.detectMarkers(img,dictionary)
                for ident,q in zip([] if raw is None else raw.ravel(),quads):
                    e=float(np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min())
                    if e < 10:
                        subthreshold.append(dict(frame=i,t=round(t,3),ident=int(ident),short_px=round(e,2),
                                                 printed=bool(printed_pattern_matches(img,q,int(ident),dictionary))))
            bench=i%40==1
            def measure(extended):
                a=time.process_time_ns()
                decoded=decode(img,dictionary,small if extended else [],tiny if extended else None)
                return (time.process_time_ns()-a)/1e6,decoded
            if bench and (i//40)%2:
                old_ms.append(measure(False)[0])
            duration,(normal,ids)=measure(True)
            if bench:
                new_ms.append(duration)
                if not (i//40)%2:old_ms.append(measure(False)[0])
            def project(u):return yaw-math.atan2(u-cx,fx)
            rows=[(project(u),e,2*u/(w-1)-1,2*v/(h-1)-1) for u,v,e,_ in small]
            named=[(ident,project(u),e,2*v/(h-1)-1) for ident,u,v,e in tiny]
            old=older.offer(t,t,origin,yaw,rows) if rows else None
            if old:
                older.commit(old);anon+=1
            edge=watch.offer(t,t,origin,yaw,rows) if rows else None
            if edge:
                watch.commit(edge)
            for ident,u,v,e in tiny:
                b=boxes.get(ident)
                native.append(dict(frame=i,stamp=round(t,3),id=ident,short_px=round(e,2),
                    pixel=[round(u,1),round(v,1)],matched_projection=bool(b is not None and b[0]-12<=u<=b[2]+12 and b[1]-12<=v<=b[3]+12)))
            if edge or not tiny:continue
            known={ident for ident,when in registered.items() if when<=t}
            match=tracker.offer(t,t,origin,yaw,named,known,watch.accepted)
            if not match:continue
            ident,req=match
            # Same-exposure pose and a separate 80 ms receive-pose replay (R2 dev p95 image age).
            cur_pos,cur_yaw,_=camera(headings,geom,t+.08)
            angle=aligned_turn(req[2],yaw,cur_yaw)
            if angle is None or math.dist(origin,cur_pos)>.22:continue
            watch.commit(req)
            hit=next(x for x in native[::-1] if x['id']==ident and x['frame']==i)
            error=math.degrees(math.remainder(req[2]-cur_yaw,math.tau))
            calls.append(dict(frame=i,stamp=round(t,3),id=ident,pixel=hit['pixel'],
                matches_same_id_projection=hit['matched_projection'],conservative_negative=is_negative,
                frozen_registration_at=registered.get(ident),initial_error_deg=round(error,2),
                signed_step_deg=round(math.degrees(angle),2),ideal_residual_deg=round(error-math.degrees(angle),2)))
    def stats(seq):
        return [round(float(np.percentile(seq,p)),3) for p in (50,95)]
    return dict(run=folder.name,window=window,frames=frames,conservative_negative_frames=neg,
                development_bare_native_subthreshold=subthreshold,eligible_native=native,anonymous_edge_requests_if_clear_R3=anon,shared_ledger_R4=len(watch.accepted),
                additional_named_glance_requests_if_clear=calls,paired_cpu_samples=len(old_ms),
                cpu_milliseconds_p50_p95=dict(R3=stats(old_ms),R4=stats(new_ms),paired_delta=stats(np.subtract(new_ms,old_ms))))


def jpeg_sensitivity(folder):
    """Measure a saved-image perturbation; raw sensor RGB cannot be reconstructed."""
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    modes={name:{'bare32':[],'eligible32':[]} for name in
           ('jpeg_to_gray','jpeg_to_bgr_then_gray','extra_jpeg_q95','extra_jpeg_q85')}
    with (folder/'frames.csv').open() as stream:
        for row in csv.DictReader(stream):
            i,t=int(row['frame']),float(row['elapsed_sim_s'])
            if not 184<=t<=205:continue
            path=str(folder/'camera'/f'{i:05d}.jpg')
            bgr=cv2.imread(path)
            images=[cv2.imread(path,cv2.IMREAD_GRAYSCALE),cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)]
            for quality in (95,85):
                ok,enc=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,quality])
                if not ok:raise ValueError('cannot encode JPEG')
                images.append(cv2.cvtColor(cv2.imdecode(enc,cv2.IMREAD_COLOR),cv2.COLOR_BGR2GRAY))
            for mode,img in zip(modes,images):
                quads,ids,_=cv2.aruco.detectMarkers(img,dictionary)
                for q,ident in zip(quads,[] if ids is None else ids.ravel()):
                    if ident!=32:continue
                    modes[mode]['bare32'].append(i)
                    edge=np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min()
                    if 8<=edge<10 and printed_pattern_matches(img,q,32,dictionary):
                        modes[mode]['eligible32'].append(i)
    return modes


def eligible_cpu(folder, frames=(972,975), repetitions=50):
    """Incremental CPU when native OpenCV actually returned an eligible tiny quad."""
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    pairs=[];checks=[]
    for frame in frames:
        img=cv2.imread(str(folder/'camera'/f'{frame:05d}.jpg'),cv2.IMREAD_GRAYSCALE)
        quads,ids,_=cv2.aruco.detectMarkers(img,dictionary)
        q=next(q for q,ident in zip(quads,ids.ravel()) if ident==32)
        for i in range(repetitions+2):
            values={}
            for extended in (bool(i%2),not bool(i%2)):
                start=time.process_time_ns()
                decode(img,dictionary,[],[] if extended else None)
                values[extended]=(time.process_time_ns()-start)/1e6
            start=time.process_time_ns()
            assert printed_pattern_matches(img,q,32,dictionary)
            gate=(time.process_time_ns()-start)/1e6
            if i>=2:pairs.append(values[True]-values[False]);checks.append(gate)
    def stats(a):return [round(float(np.percentile(a,p)),3) for p in (50,95)]
    return dict(frames=frames,alternating_full_decode_pairs=len(pairs),paired_delta_ms_p50_p95=stats(pairs),
                printed_pattern_check_ms_p50_p95=stats(checks))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('geometry',type=Path);p.add_argument('development',type=Path)
    p.add_argument('historical',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();cv2.setNumThreads(1);g=json.loads(args.geometry.read_text())
    results=[run(args.development,g,(184,205))];print('development',results[0]['additional_named_glance_requests_if_clear'],flush=True)
    for f in sorted(args.historical.glob('v151-rescue360-track-*/')):
        results.append(run(f,g));print(f.name,results[-1]['frames'],results[-1]['additional_named_glance_requests_if_clear'],flush=True)
    assert len(results)==4
    out=dict(opencv=cv2.__version__,scope='Development excerpt plus three complete archived rescue360 missions. '
             'JPEG is not live raw; poses use truth interpolated to capture and capture+80ms as passive TF proxies. '
             'Live hazard, near-pass, accepted actions, callback ordering and actual retarget cost cannot be replayed. '
             'Registered IDs excluded using frozen registration times; other runtime guards assumed clear. '
             'GT is used only to label image coordinates after detection; 12px tolerance; conservative negative = no '
             'projected GT box within the image extended by 64px horizontally. Runtime does not consume GT or use range. '
             'CPU measurements use alternating paired old/new full native decode each fortieth frame on single-threaded Mac. '
             'Prior R3 tiny boundary relook and new typed cue share the same 2-per-mission and 30s cooldown.',
             development_saved_pixel_sensitivity=jpeg_sensitivity(args.development),
             development_eligible_cpu=eligible_cpu(args.development),runs=results)
    args.output.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
