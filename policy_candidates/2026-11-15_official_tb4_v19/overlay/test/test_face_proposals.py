import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import boundary_faces, options, visible
from osp_mission.face_proposals import clear_segments, select_faces


def test_batched_visibility_matches_existing_individual_rays():
    rng=np.random.default_rng(49)
    data=rng.choice([0,100,-1],size=(35,45),p=[.85,.10,.05])
    y=rng.uniform(-2,37,500);x=rng.uniform(-2,47,500)
    ty=rng.uniform(-2,37,500);tx=rng.uniform(-2,47,500)
    expected=[visible(data,a,b,np.array([c]),np.array([d]))[0]
              for a,b,c,d in zip(y,x,ty,tx)]
    assert np.array_equal(clear_segments(data,y,x,ty,tx),expected)


def test_sparse_reachable_faces_survive_a_large_infeasible_population():
    data=np.zeros((80,100),dtype=np.int8);data[:,50]=100
    faces=boundary_faces(data,.05)
    valid=np.zeros_like(data,dtype=bool);valid[37:42,40:43]=True
    distance=np.ones(data.shape)
    indices=select_faces(data,.05,faces,valid,distance,(.4,),limit=3)
    assert len(indices)==3 and len(set(indices))==3
    assert all(faces['nx'][i]<0 for i in indices)
    assert all(37<=faces['y'][i]<42 for i in indices)
    valid[:]=False
    assert len(select_faces(data,.05,faces,valid,distance,(.4,)))==0


def test_unknown_sightline_is_rejected_even_with_a_valid_destination():
    data=np.zeros((30,30),dtype=np.int8);data[:,15]=100
    faces=boundary_faces(data,.05)
    valid=np.zeros_like(data,dtype=bool);valid[10:20,7]=True
    data[:,10]=-1
    assert not len(select_faces(data,.05,faces,valid,np.ones(data.shape),(.4,)))


def test_opposite_thin_wall_sides_keep_separate_proposals():
    data=np.zeros((30,30),dtype=np.int8);data[15,15]=100
    faces=boundary_faces(data,.05)
    keep=np.isin(faces['side'],[1,3]);faces={k:v[keep] for k,v in faces.items()}
    indices=select_faces(data,.05,faces,data==0,np.ones(data.shape),(.4,),limit=2)
    assert set(faces['side'][indices])=={1,3}


def test_valid_legacy_faces_remain_first_and_keep_their_order():
    data=np.zeros((80,100),dtype=np.int8);data[:,50]=100
    faces=boundary_faces(data,.05)
    valid=np.zeros_like(data,dtype=bool);valid[:,10:45]=True
    distance=np.ones(data.shape)
    eligible=select_faces(data,.05,faces,valid,distance,(.4,),limit=len(faces['y']))
    legacy=np.linspace(0,len(faces['y'])-1,48,dtype=int)
    retained=legacy[np.isin(legacy,eligible)]
    actual=select_faces(data,.05,faces,valid,distance,(.4,))
    assert 0<len(retained)<48 and len(actual)==48
    assert np.array_equal(actual[:len(retained)],retained)
    assert set(actual)<=set(eligible)


@pytest.mark.parametrize('rotation',range(4))
@pytest.mark.parametrize('resolution',[.025,.05])
def test_rotated_clutter_keeps_body_clearance_budget_and_camera_evidence(rotation,resolution):
    # Two isolated obstacles, a thin partition and a dead-end, with no target input.
    n=round(6/resolution);grid=np.zeros((n,n),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[round(2/resolution):round(2.5/resolution),round(2/resolution):round(2.5/resolution)]=100
    grid[round(4/resolution):round(4.5/resolution),round(4/resolution):round(4.5/resolution)]=100
    grid[round(3/resolution):round(5/resolution),round(3/resolution)]=100
    grid[round(5/resolution),round(3/resolution):round(4/resolution)]=100
    grid=np.rot90(grid,rotation).copy();viewed=np.zeros((4,)+grid.shape,dtype=bool)
    goals,_=options(grid,resolution,(0,0),(1.,1.),viewed=viewed,inspection_focal=221.765,collect_faces=True)
    directed=[g for g in goals if g.get('face_inspection')]
    clearance=distance_transform_edt(grid<=50)*resolution
    assert directed and len(directed)<=96
    assert all(clearance[g['cell']]>=.32 for g in goals)
    assert all(g['features'][0]>0 for g in directed)
    assert not viewed.any()  # Generating a goal cannot certify an image observation.
    assert len({g['cell'] for g in goals})==len(goals)
