"""Screen native-downsampled clips with the unchanged v1.2 decoder."""
import csv,json,math,random,sys,time
from pathlib import Path
import cv2
import numpy as np
from osp_mission.marker_pair import decode

SIZES=(250,320,400,480,640)
RATES=(5,10,15)


def selected_indices(times,fps,phase):
    next_time=times[0]+phase/fps
    selected=[]
    for i,t in enumerate(times):
        if t+1e-9>=next_time:
            selected.append(i)
            while next_time<=t+1e-9:next_time+=1/fps
    return selected


def windows(times):
    result=[]
    for t in times:
        if not result or t-result[-1][-1]>.21:result.append([t])
        else:result[-1].append(t)
    return [(group[0],group[-1]) for group in result]


def evaluate(records,times,expected,fps,phase,oracle):
    chosen=selected_indices(times,fps,phase)
    readable={ident:[] for ident in expected};wrong=[]
    for i in chosen:
        for ident,edge in records[i]['ids'].items():
            if ident not in expected:wrong.append((times[i],ident))
            elif edge>=18:readable[ident].append(times[i])
    hit=total=0
    for ident,ranges in oracle.items():
        for lo,hi in ranges:
            total+=1
            hit+=any(lo-.035<=t<=hi+.035 for t in readable[ident])
    return dict(phase=phase,frames=len(chosen),readable_ids=[i for i,v in readable.items() if v],
        first_readable_s={i:v[0] for i,v in readable.items() if v},readable_frames={i:len(v) for i,v in readable.items()},
        native_visible_episodes=total,episodes_sampled_and_readable=hit,unexpected_raw_id_frames=len(wrong))


def process_clip(run,output):
    frame_rows=list(csv.DictReader((run/'frames.csv').open()))
    camera=json.loads((run/'camera_metrics.json').read_text())
    assert camera['width']==camera['height']==640
    assert camera['input_fps_sim']>=13.5,'Reference capture dropped too many frames for a 15 FPS comparison'
    expected={int(t['id']) for t in json.loads((run/'scene.json').read_text())['targets']}
    times=[float(row['elapsed_sim_s']) for row in frame_rows]
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    records={size:[] for size in SIZES};timings={size:[] for size in SIZES}
    rng=random.Random(1120)
    for index,row in enumerate(frame_rows):
        original=cv2.imread(str(run/'raw_frames'/f'{int(row["frame"]):05d}.png'),cv2.IMREAD_GRAYSCALE)
        assert original is not None and original.shape==(640,640)
        order=list(SIZES);rng.shuffle(order)
        for size in order:
            image=original if size==640 else cv2.resize(original,(size,size),interpolation=cv2.INTER_AREA)
            before=time.perf_counter();small=[]
            corners,ids=decode(image,dictionary,small)
            elapsed=(time.perf_counter()-before)*1000
            if index>=3:timings[size].append(elapsed)
            found={}
            if ids is not None:
                for corner,ident in zip(corners,ids.ravel()):
                    found[int(ident)]=max(found.get(int(ident),0.),float(np.linalg.norm(corner[0]-np.roll(corner[0],1,axis=0),axis=1).min()))
            records[size].append(dict(ids=found,small_candidates=len(small)))
        if index%100==0:print('screen',run.name,index,'/',len(frame_rows),flush=True)
    oracle={ident:windows([t for t,r in zip(times,records[640]) if r['ids'].get(ident,0)>=18]) for ident in expected}
    rows=[]
    for size in SIZES:
        for fps in RATES:
            phases=[evaluate(records[size],times,expected,fps,phase,oracle) for phase in (0.,1/3,2/3)]
            q=np.asarray(timings[size]);proxy=np.mean([len(p['readable_ids']) for p in phases])
            episodes=sum(p['native_visible_episodes'] for p in phases)
            rows.append(dict(size=size,fps=fps,raw_rgb_mbps=size*size*3*fps*8/1e6,
                decode_median_ms=float(np.median(q)),decode_p95_ms=float(np.percentile(q,95)),
                decode_p95_fraction_of_frame_budget=float(np.percentile(q,95)*fps/1000),
                mean_readable_id_count=float(proxy),episode_capture_fraction=sum(p['episodes_sampled_and_readable'] for p in phases)/max(1,episodes),
                unexpected_raw_id_frames=sum(p['unexpected_raw_id_frames'] for p in phases),phases=phases))
    report=dict(run=run.name,role='selection' if 'm112021' in run.name else 'held_out_replay',
        native_camera=camera,opencv_threads=cv2.getNumThreads(),frames=len(frame_rows),expected_ids=sorted(expected),rows=rows,
        limits='Decode-only timing includes v1.2 small-wall proposal generation, not TF/range/consensus or full ROS callback; readable IDs are not final registration')
    output.write_text(json.dumps(report,indent=2)+'\n')
    return report


def choose(selection):
    rows=selection['rows']
    rank=lambda r:(r['mean_readable_id_count'],r['episode_capture_fraction'],-r['unexpected_raw_id_frames'],-r['raw_rgb_mbps'])
    usable=[r for r in rows if r['decode_p95_fraction_of_frame_budget']<=.6]
    if not usable:usable=rows
    picks=[[480,5]]  # Explicitly test the user's low-FPS proposal.
    for predicate in (lambda r:r['raw_rgb_mbps']<=60,
                      lambda r:r['fps']>=10 and r['raw_rgb_mbps']<=60):
        eligible=[r for r in usable if predicate(r) and [r['size'],r['fps']] not in picks and [r['size'],r['fps']]!=[250,10]]
        if eligible:
            best=max(eligible,key=rank);picks.append([best['size'],best['fps']])
    return picks


if __name__=='__main__':
    root=Path(sys.argv[1]);reports=[]
    for i,name in enumerate(json.loads((root/'reference_clips.json').read_text())):
        reports.append(process_clip(Path(name),root/f'replay-map-{i+1}.json'))
    result=dict(settings=15,replay_clips=len(reports),sampling_phases=3,derived_replay_cases=len(reports)*15*3,
        selection_clip=reports[0]['run'],held_out_replay_clip=reports[1]['run'],native_candidates=choose(reports[0]),
        selection_rule='Preserve 480/5; choose two distinct alternatives under 60 Mbps and decode headroom, using only map 112021',
        no_physical_safety_certification=True)
    (root/'screening.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
