"""Spread a bounded set of feasible views over observed, uninspected faces.

Only occupancy, reachability and calibration-derived standoffs enter here.
Choosing a viewpoint never marks a surface as inspected.
"""
import numpy as np


def clear_segments(data, y, x, to_y, to_x):
    """Batch the existing single-ray free-space test without changing its samples."""
    lengths=np.hypot(to_y-y,to_x-x)
    steps=np.maximum(4,(lengths*3).astype(int)+1)
    stop=np.maximum(0.,1.-.10/np.maximum(lengths,1e-9))
    result=np.zeros(len(y),dtype=bool)
    for count in np.unique(steps):
        group=np.flatnonzero(steps==count)
        for begin in range(0,len(group),256):
            ids=group[begin:begin+256]
            fractions=stop[ids,None]*np.linspace(0,1,int(count))
            ys=np.floor(y[ids,None]+(to_y[ids]-y[ids])[:,None]*fractions).astype(int)
            xs=np.floor(x[ids,None]+(to_x[ids]-x[ids])[:,None]*fractions).astype(int)
            inside=(ys>=0)&(ys<data.shape[0])&(xs>=0)&(xs<data.shape[1])
            result[ids]=np.all(inside&(data[ys.clip(0,data.shape[0]-1),xs.clip(0,data.shape[1]-1)]==0),axis=1)
    return result


def select_faces(data,resolution,faces,valid,distance,standoffs,limit=48):
    """Choose at most ``limit`` face indices, after body/path/LOS feasibility.

    Keep useful legacy proposals in their original order, then fill only wasted
    slots by spreading in position and outward normal. Opposite thin-wall sides
    remain distinct. The existing policy still ranks the resulting goals.
    """
    n=len(faces['y'])
    if n==0 or not standoffs or limit<=0:return np.empty(0,dtype=int)
    nearest=np.full(n,np.inf)
    for standoff in standoffs:
        y=np.floor(faces['py']+faces['ny']*standoff/resolution).astype(int)
        x=np.floor(faces['px']+faces['nx']*standoff/resolution).astype(int)
        ids=np.flatnonzero((y>=0)&(y<data.shape[0])&(x>=0)&(x<data.shape[1]))
        ids=ids[valid[y[ids],x[ids]]&np.isfinite(distance[y[ids],x[ids]])]
        ids=ids[clear_segments(data,y[ids]+.5,x[ids]+.5,faces['py'][ids],faces['px'][ids])]
        nearest[ids]=np.minimum(nearest[ids],distance[y[ids],x[ids]])
    eligible=np.flatnonzero(np.isfinite(nearest))
    if not len(eligible):return eligible
    scale=min(1.,max(standoffs))
    coordinates=np.column_stack((faces['px'][eligible]*resolution,faces['py'][eligible]*resolution,
                                 faces['nx'][eligible]*scale,faces['ny'][eligible]*scale))
    legacy=np.linspace(0,n-1,min(limit,n),dtype=int)
    chosen=np.flatnonzero(np.isin(eligible,legacy)).tolist()
    if not chosen:chosen=[int(np.argmin(nearest[eligible]))]
    spacing=np.full(len(eligible),np.inf)
    for next_index in chosen:
        delta=coordinates-coordinates[next_index]
        spacing=np.minimum(spacing,np.sum(delta*delta,axis=1))
    for _ in range(min(limit,len(eligible))-len(chosen)):
        spacing[chosen]=-1.
        next_index=int(np.argmax(spacing))
        chosen.append(next_index)
        delta=coordinates-coordinates[next_index]
        spacing=np.minimum(spacing,np.sum(delta*delta,axis=1))
    return eligible[np.asarray(chosen)]
