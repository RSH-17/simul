"""Online frontier clusters plus camera-surface set coverage over observed maps.

Nav2 owns path execution and collision avoidance. No scene/target information
enters this module. Planned coverage is never marked as sensor observation.
"""
import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label
from scipy.sparse.csgraph import dijkstra

from osp_mission import edge_policy
from osp_mission.frontier_support import supported_unknown
from osp_mission.policy import graph, wrap
from osp_mission.time_budget import choose as legacy_choose


def clustered_frontiers(data, resolution, origin, robot, heading):
    """Choose body-clear standoffs with a free sightline to each unknown edge."""
    free=data==0
    y,x=np.floor((np.asarray(robot)-origin)/resolution).astype(int)[::-1]
    if not (0<=y<data.shape[0] and 0<=x<data.shape[1]) or not free[y,x]:return []
    clearance=distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*resolution
    safe=free&(clearance>=.32)
    if not safe[y,x]:return []  # The unchanged narrow-start controller owns this case.
    distances=dijkstra(graph(safe,resolution),directed=False,indices=y*data.shape[1]+x).reshape(data.shape)
    reachable=free.copy()
    groups,_=label(free);reachable &= groups==groups[y,x]
    edge=reachable&binary_dilation(supported_unknown(data,resolution))
    clusters,count=label(edge,structure=np.ones((3,3)))
    goals=[]
    for ident in range(1,count+1):
        ys,xs=np.where(clusters==ident)
        if len(ys)*resolution<.25:continue
        # Keep multiple spread anchors for a long bent frontier; a centroid may
        # lie on the wrong side of a wall. Bound the expensive LOS checks.
        anchors=np.linspace(0,len(ys)-1,min(8,len(ys)),dtype=int)
        proposals=[]
        for index in anchors:
            ey,ex=ys[index],xs[index];radius=int(math.ceil(.9/resolution))
            y0,y1=max(0,ey-radius),min(data.shape[0],ey+radius+1)
            x0,x1=max(0,ex-radius),min(data.shape[1],ex+radius+1)
            cy,cx=np.where(safe[y0:y1,x0:x1]&np.isfinite(distances[y0:y1,x0:x1]))
            cy+=y0;cx+=x0
            separation=np.hypot(cy-ey,cx-ex)*resolution
            valid=(separation>=.35)&(separation<=.9)&(distances[cy,cx]>.45)
            cy,cx=cy[valid],cx[valid]
            order=np.argsort(distances[cy,cx]+np.abs(separation[valid]-.55))[:12]
            for k in order:
                yy,xx=int(cy[k]),int(cx[k])
                if not edge_policy.visible(data,yy+.5,xx+.5,np.array([ey+.5]),np.array([ex+.5]))[0]:continue
                yaw=math.atan2(ey-yy,ex-xx)
                proposals.append(dict(x=origin[0]+(xx+.5)*resolution,y=origin[1]+(yy+.5)*resolution,
                    yaw=yaw,scan_angle=0.,cell=(yy,xx),distance=float(distances[yy,xx]),
                    frontier_cluster=ident,coverage_kind='frontier',
                    features=np.array([0.,min(5.,len(ys)*resolution),distances[yy,xx],abs(wrap(yaw-heading)),0.])))
                break
        if proposals:goals.append(min(proposals,key=lambda g:g['distance']+.2*g['features'][3]))
    return goals


def options(data,resolution,origin,robot,heading=0.,visits=(),viewed=None,return_paths=False,**kwargs):
    if not np.isfinite([resolution,*origin,*robot,heading]).all() or resolution<=0:return [],None
    # Existing feasibility-first directed face proposals retain both sides of
    # thin obstacles. Their coverage sets contain only not-yet-observed faces.
    kwargs.pop('collect_faces',None)
    goals,previous=edge_policy.options(data,resolution,origin,robot,heading,visits,viewed,
        return_paths=return_paths,collect_faces=True,**kwargs)
    if goals and all(g.get('start_escape') for g in goals):return goals,previous
    coverage=[]
    for original in goals:
        if original['features'][0]<=0:continue
        candidate=dict(original);candidate['features']=original['features'].copy()
        # Local blurred frontier gain no longer competes inside coverage goals.
        candidate['features'][1]=0.;candidate['coverage_kind']='surface'
        coverage.append(candidate)
    return coverage+clustered_frontiers(data,resolution,origin,robot,heading),previous


def choose(candidates,weights,remaining_seconds):
    if not candidates:return None
    if all(g.get('start_escape') for g in candidates):return legacy_choose(candidates,weights,remaining_seconds)
    eligible=[]
    for g in candidates:
        f=g['features'];time=f[2]/.20+f[3]/.45+abs(g.get('scan_angle',0.))/.45+3.5
        if not math.isfinite(time) or time>remaining_seconds:continue
        eligible.append((g,time))
    surfaces=[(g,t) for g,t in eligible if g.get('coverage_kind')=='surface']
    frontiers=[(g,t) for g,t in eligible if g.get('coverage_kind')=='frontier']
    # Opportunistic local coverage prevents driving past readable surfaces.
    # Long trips to old surfaces yield to a reachable exploration frontier.
    local=[(g,t) for g,t in surfaces if g['distance']<=2. and g['features'][0]>=.15]
    pool=local or (frontiers if remaining_seconds>90. and frontiers else surfaces or frontiers)
    if not pool:return None
    def utility(item):
        g,time=item;surface,frontier,_,_,revisit=g['features']
        gain=surface if g.get('coverage_kind')=='surface' else frontier
        return gain/time-weights[4]*revisit*.1
    return max(pool,key=utility)[0]
