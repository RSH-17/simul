import math
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from osp_mission.frontier_coverage import clustered_frontiers,options,choose


@pytest.mark.parametrize('resolution',[.025,.05,.1])
def test_frontier_goals_are_known_free_clear_reachable_and_face_unknown(resolution):
    n=round(8/resolution);d=np.full((n,n),-1,dtype=np.int8)
    d[1:-1,1:round(5/resolution)]=0;d[0,:]=100;d[-1,:]=100;d[:,0]=100
    goals=clustered_frontiers(d,resolution,(0.,0.),(2.,4.),0.)
    assert goals
    clear=distance_transform_edt(np.pad(d==0,1))[1:-1,1:-1]*resolution
    for g in goals:
        assert d[g['cell']]==0 and clear[g['cell']]>=.32
        assert g['x']<5 and abs(g['yaw'])<math.pi/2


def test_unreachable_room_frontier_is_excluded():
    d=np.full((160,200),100,dtype=np.int8);d[5:155,5:75]=0;d[5:155,90:150]=0;d[5:155,150:]=-1
    assert not clustered_frontiers(d,.05,(0.,0.),(2.,4.),0.)


def test_raster_holes_do_not_create_exploration_targets():
    d=np.zeros((120,160),dtype=np.int8);d[[0,-1],:]=100;d[:,[0,-1]]=100;d[60,80]=-1
    assert not clustered_frontiers(d,.05,(0.,0.),(2.,3.),0.)


def goal(kind,distance=1.,gain=.4):
    return dict(x=distance,y=0.,yaw=0.,distance=distance,scan_angle=math.pi/3,
        coverage_kind=kind,features=np.array([gain if kind=='surface' else 0.,
        3. if kind=='frontier' else 0.,distance,0.,0.]))


def test_near_surface_is_inspected_before_leaving_for_new_room():
    surface=goal('surface');frontier=goal('frontier',4.)
    assert choose([frontier,surface],np.ones(5),600.) is surface
    assert surface['features'][0]==.4  # Choosing is not camera evidence.


def test_far_old_surface_does_not_block_exploration_and_deadline_returns_to_coverage():
    surface=goal('surface',4.);frontier=goal('frontier',5.)
    assert choose([surface,frontier],np.ones(5),600.) is frontier
    assert choose([surface,frontier],np.ones(5),80.) is surface
    assert choose([surface,frontier],np.ones(5),3.) is None


def test_full_map_still_produces_surface_coverage_without_frontiers():
    d=np.zeros((120,160),dtype=np.int8);d[[0,-1],:]=100;d[:,[0,-1]]=100;d[50:70,70:90]=100
    before=d.copy();viewed=np.zeros((4,)+d.shape,dtype=bool)
    goals,_=options(d,.05,(0.,0.),(2.,3.),viewed=viewed,inspection_focal=221.765)
    assert any(g.get('coverage_kind')=='surface' for g in goals)
    assert not any(g.get('coverage_kind')=='frontier' for g in goals)
    assert np.array_equal(d,before) and not viewed.any()


def test_narrow_start_preserves_body_clear_exit_instead_of_forcing_surface_sweep():
    d=np.full((160,200),100,dtype=np.int8);d[75:85,5:100]=0;d[20:140,100:190]=0;d[20:140,190:]=-1
    goals,_=options(d,.05,(0.,0.),(1.,4.),inspection_focal=221.765)
    assert goals and all(g.get('start_escape') for g in goals)
    assert all(g['scan_angle']==0 for g in goals)


def test_mission_wires_hybrid_generation_and_selection_to_nav2():
    import inspect
    from osp_mission.mission import Mission
    source=inspect.getsource(Mission)
    assert 'self.policy_options=frontier_coverage.options if self.edge_mode' in source
    assert '(frontier_coverage.choose if self.edge_mode else choose_before_deadline)' in source
    assert "self.send(self.nav,g,'nav')" in source
