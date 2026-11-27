"""Rank unexplored regions without rewarding isolated scan raster gaps.

This changes post-departure information gain only. It never marks unknown cells
free, changes planning/collision grids, or filters crowded-start waypoints.
"""
import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label, uniform_filter


def supported_unknown(data,resolution):
    """Keep unknown regions with area and width beyond a few lidar grid cells.

    The 0.2 m core diameter is deliberately smaller than the robot: an unknown
    entrance can be narrow while its connected region extends into a room.
    These are information-ranking thresholds, not assertions of traversability.
    """
    unknown=data<0
    regions,count=label(unknown)
    if not count:return unknown
    area=np.bincount(regions.ravel())*resolution**2
    has_core=np.zeros(count+1,dtype=bool)
    has_core[np.unique(regions[distance_transform_edt(unknown)*resolution>=.10])]=True
    supported=(area>=math.pi*.20**2)&has_core;supported[0]=False
    return unknown&supported[regions]


def exploration_gain(data,resolution,reachable):
    edge=reachable&binary_dilation(supported_unknown(data,resolution))
    groups,count=label(edge,structure=np.ones((3,3)))
    if count:
        large=np.bincount(groups.ravel())*resolution>=.25;large[0]=False
        edge&=large[groups]
    size=max(3,int(3/resolution)|1)
    length=uniform_filter(edge.astype(float),size=size,mode='constant')*size**2*resolution
    return np.clip(length/3.,0.,1.)
