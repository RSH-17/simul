"""Starting an already undocked robot must not send a rejected undock request."""
from types import SimpleNamespace as S

import pytest

from osp_mission.mission import Mission


@pytest.mark.parametrize('dock,expected_start,expected_action', [
    (None,False,False), ({'stamp':1.,'is_docked':False},False,False),
    ({'stamp':10.,'is_docked':False},True,False),
    ({'stamp':10.,'is_docked':True},True,True)])
def test_start_uses_fresh_dock_state(dock,expected_start,expected_action):
    actions=[]
    owner=S(state='idle',ready=lambda:True,p=lambda name:True,
        sensor_fresh=lambda stamp:10.-stamp<=2.,dock_status=dock,now=lambda:10.,
        undock=S(server_is_ready=lambda:True),send=lambda *args:actions.append(args),
        capture_dock_anchor=lambda:True)
    response=S(success=False,message='')
    Mission.start(owner,None,response)
    assert response.success is expected_start
    assert bool(actions) is expected_action
    if expected_start and not expected_action:
        assert owner.undock_diagnostics['reason']=='reported_undocked'
