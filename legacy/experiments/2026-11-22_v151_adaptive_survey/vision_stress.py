"""Deterministic camera-only stress tests. Run with staged R2's src/osp_mission on PYTHONPATH."""
import argparse
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from osp_mission.hybrid_geometry import preview_geometry
from osp_mission.landmark_identity import resolved_ids
from osp_mission.marker_pair import decode, SeparateMarkers
from osp_mission.small_wall_cues import anonymous_center, usable_named_quad
from osp_mission.thorough_borders import borders


BACKGROUNDS = ('plain', 'gradient_noise', 'furniture_lines', 'checker_fabric')
VIEWS = ('frontal','skew','lateral_clip')
TAGS = ('clear','occluded','motion_blur','far')
SIZE = 480
SEED = 11220431


def background(kind, rng):
    y,x=np.indices((SIZE,SIZE))
    if kind == 'plain':
        return np.full((SIZE,SIZE),184,np.uint8)
    if kind == 'checker_fabric':
        return np.uint8(125+45*((x//8+y//9)%2)+14*((x+y)%4==0))
    image=np.uint8(np.clip(100+.24*x+21*np.sin(y/29)+rng.normal(0,7,x.shape),0,255))
    if kind == 'furniture_lines':
        for _ in range(22):
            start=rng.integers(0,SIZE,2); end=start+rng.integers(-90,91,2)
            cv2.line(image,tuple(start),tuple(end),int(rng.integers(34,166)),int(rng.integers(2,6)))
    return image


def perspective(center, size, view, rng):
    corners=np.float32([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
    if view=='skew':
        corners[:,0]*=rng.uniform(.48,.76)
        corners[1:3,1]*=rng.uniform(.66,.85)
    angle=rng.uniform(-.20,.20) if view!='frontal' else rng.uniform(-.04,.04)
    rotation=np.float32([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    return np.float32(corners@rotation.T*size+center)


def paint(image, cells, quad, blur):
    patch=np.full((126,126),230,np.uint8)
    patch[21:105,21:105]=cv2.resize(cells,(84,84),interpolation=cv2.INTER_NEAREST)
    source=np.float32([[21,21],[104,21],[104,104],[21,104]])
    matrix=cv2.getPerspectiveTransform(source,quad)
    content=cv2.warpPerspective(patch,matrix,(SIZE,SIZE),flags=cv2.INTER_LINEAR)
    if blur:
        content=cv2.blur(content,(5,1))
    support=cv2.warpPerspective(np.full_like(patch,255),matrix,(SIZE,SIZE),flags=cv2.INTER_NEAREST)>0
    image[support]=content[support]


def edge(quad):
    return float(np.linalg.norm(quad.reshape(4,2)-np.roll(quad.reshape(4,2),1,axis=0),axis=1).min())


def includes(q,hit):
    return cv2.pointPolygonTest(q,tuple(map(float,hit[:2])),False)>=0


def stress(parent, calibration):
    cv2.setNumThreads(1)
    spec=importlib.util.spec_from_file_location('frozen_marker_pair',parent/'marker_pair.py')
    old=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    rng=np.random.default_rng(SEED)
    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    groups={}
    ids_drawn=set()
    for bg in BACKGROUNDS:
        for view in VIEWS:
            for condition in TAGS:
                counter=groups.setdefault(f'{view}/{condition}',Counter())
                for trial in range(4):
                    ids=[7,103] if trial<2 else rng.choice(len(dic.bytesList),2,replace=False).tolist()
                    ids_drawn.update(ids)
                    image=background(bg,rng)
                    s1=rng.uniform(30,50)
                    s2=rng.uniform(6,10) if condition=='far' else rng.uniform(22,45)
                    y=float(rng.integers(90,390))
                    right=SIZE-rng.uniform(.05,.32)*s2 if view=='lateral_clip' else rng.uniform(205,370)
                    q2=perspective([right,y],s2,view,rng)
                    q1=perspective([0.,y],s1,view,rng)
                    # The inner printed regions are disjoint with a measured positive image gap.
                    q1[:,0]+=q2[:,0].min()-rng.uniform(6,14)-q1[:,0].max()
                    paint(image,draw(dic,ids[0],7),q1,False)
                    paint(image,draw(dic,ids[1],7),q2,condition=='motion_blur')
                    if condition=='occluded':
                        cx=int(q2[:,0].mean()+.22*s2)
                        cv2.rectangle(image,(cx,int(y-s2*.65)),(cx+max(2,round(s2*.21)),int(y+s2*.65)),int(rng.integers(35,120)),-1)
                    baseline=[]
                    _,old_ids=old.decode(image,dic,baseline)
                    tiny=[]
                    corners,read=decode(image,dic,tiny)
                    old_ids=[] if old_ids is None else old_ids.ravel().tolist()
                    names=[] if read is None else read.ravel().tolist()
                    known=[q for q in corners if usable_named_quad(q,SIZE,SIZE)]
                    accepted={n for n,q in zip(names,corners) if usable_named_quad(q,SIZE,SIZE) and edge(q)>=18}
                    proposals=borders(image)
                    anonymous=[h for h in proposals if anonymous_center(h,known)]
                    counter['frames']+=1
                    counter['frames_with_spurious_decoded_id']+=bool(set(names)-set(ids))
                    counter['frames_candidate_changed_decoded_ids']+=(sorted(old_ids)!=sorted(names))
                    counter['both_ids_ge18_and_interior']+=int(set(ids)<=accepted)
                    if ids[0] in accepted and ids[1] not in accepted:
                        counter['first_id_readable_second_not_readable']+=1
                        hit_pre=any(includes(q2,h) for h in proposals)
                        hit_post=any(includes(q2,h) for h in anonymous)
                        hit_tiny=any(includes(q2,h) for h in tiny)
                        # When the second marker has already decoded provisionally, local suppression is correct.
                        if ids[1] not in names:
                            counter['second_entirely_undecoded']+=1
                            counter['second_undecoded_raw_outline']+=hit_pre
                            counter['second_undecoded_retained_outline']+=hit_post
                            counter['second_undecoded_with_tiny_or_outline']+=(hit_tiny or hit_post)
                            counter['separate_outline_wrongly_suppressed']+=(hit_pre and not hit_post)
    negatives={}
    for bg in BACKGROUNDS:
        tally=Counter()
        for i in range(24):
            image=background(bg,rng)
            # Deliberate hard negatives with dark rim + random alternating interior.
            # Every row is checkerboard: neither rotation contains valid Original row words.
            if i%3:
                checker=np.zeros((7,7),np.uint8)
                checker[1:6,1:6]=((np.indices((5,5)).sum(axis=0)+i)%2)*225
                side=rng.uniform(6,11) if i%3==1 else rng.uniform(12,33)
                paint(image,checker,perspective(rng.uniform(35,445,2),side,'frontal',rng),False)
            before=[]; _,oid=old.decode(image,dic,before)
            after=[]; _,nid=decode(image,dic,after)
            tally['frames']+=1
            tally['frozen_any_decoded_id']+=oid is not None
            tally['r2_any_decoded_id']+=nid is not None
            tally['frozen_any_tiny']+=bool(before)
            tally['r2_any_tiny']+=bool(after)
            tally['generic_outline_frames']+=bool(borders(image))
        negatives[bg]=dict(tally)
    info=SimpleNamespace(width=480,height=480,header=SimpleNamespace(frame_id=calibration['frame']),k=calibration['k'])
    code_k=list(preview_geometry(info).k)
    # Isolate the temporal identity guard with two observed nonoverlapping image quads.
    q1=np.float32([[[10,10],[40,10],[40,40],[10,40]]])
    q2=q1+np.float32([38,0]); limits=[]
    for separation in (0.,.1,.13):
        histories={a:[(i,np.float64([x,0,0])) for i in range(3)] for a,x in [(7,0.),(103,separation)]}
        for seen in (0,2,3):
            sep=SeparateMarkers()
            for i in range(seen):
                sep.observe(i,{7:q1,103:q2})
            accepted,conflict=resolved_ids(histories,separate=sep)
            limits.append(dict(map_spacing_m=separation,simultaneous_clear_frames=seen,
                               resolved=sorted(accepted),conflict=bool(conflict)))
    return dict(seed=SEED,opencv=cv2.__version__,rig=dict(pixels=SIZE,fixture_fx=calibration['k'][0],
                policy_fx=code_k[0],cx=code_k[2]),tagged=dict(groups=groups,distinct_dictionary_ids=len(ids_drawn)),
                printed_and_texture_negatives=negatives,identity_guard=limits,
                limits='Synthetic planar image tests; counted decoder outputs and >=18px interior admission '
                'separately from registry logic. They do not measure TF/depth, time-consensus in actual motion, '
                'registration, safe approach, map accuracy, or navigation. Static image-level negatives '
                'are not counts of false turns. 480 simulation calibration does not identify a physical camera profile.')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('frozen_package',type=Path)
    parser.add_argument('calibration',type=Path)
    parser.add_argument('output',type=Path)
    a=parser.parse_args()
    r=stress(a.frozen_package,json.loads(a.calibration.read_text()))
    a.output.write_text(json.dumps(r,indent=2)+'\n')
