import json
import math
from types import SimpleNamespace as S

import numpy as np
import pytest

from osp_mission import edge_policy
from osp_mission.mission import Mission
from osp_mission.time_budget import choose
from osp_mission.policy_audit import save


def policy(tmp_path,settings):
    path=tmp_path/'policy.json'
    path.write_text(json.dumps(dict(planner='edge',parameters=[1,2,1,1,1],settings=settings)))
    return path


def test_edge_mission_reads_deadline_setting_and_keeps_legacy_default(tmp_path):
    node=S(p=lambda k:'edge',policy_load=edge_policy.load_policy)
    weights,settings=Mission.load_configuration(node,policy(tmp_path,{'deadline_aware':True}))
    assert settings=={'deadline_aware':True}
    assert 'view_standoff' not in settings and weights.tolist()==[1,2,1,1,1]
    assert Mission.load_configuration(node,policy(tmp_path,{}))[1]=={'deadline_aware':False}


@pytest.mark.parametrize('settings',[{'deadline_aware':1},{'deadline_aware':'true'},[],{'view_standoff':.1}])
def test_invalid_settings_cannot_silently_change_clearance_or_enable_a_policy(tmp_path,settings):
    with pytest.raises(ValueError):edge_policy.load_configuration(policy(tmp_path,settings))


def test_deadline_discount_keeps_original_gain_metadata_and_audit_identity(tmp_path):
    def candidate(x,features):
        return dict(x=x,y=1.,yaw=0.,scan_angle=math.pi/3,face_inspection=True,features=np.array(features))
    far=candidate(8.,[0.,5.,8.,0.,0.]);near=candidate(1.,[.8,.1,1.,0.,0.])
    old_far=far['features'].copy();old_near=near['features'].copy()
    chosen=choose([far,near],np.ones(5),20.)
    assert chosen is near and chosen['face_inspection']
    np.testing.assert_array_equal(far['features'],old_far)
    np.testing.assert_array_equal(near['features'],old_near)
    save(tmp_path,0,580.,'edge',np.zeros((4,4)),.1,(0,0),(1,1),0,[],None,
         np.ones(5),[far,near],chosen)
    with np.load(tmp_path/'policy_decisions/0000.npz') as d:
        assert int(d['selected_index'])==1
        assert d['candidates'][1,5]==.1


def test_nominal_long_trip_is_rejected_without_banning_short_transit():
    far=dict(scan_angle=2*math.pi/3,features=np.array([.9,1.2083,36.2694,1.6786,0.]))
    near=dict(scan_angle=math.pi/3,features=np.array([.8,0.,4.6991,2.7719,.0339]))
    weights=np.array([1.,2.409531985,.48895122,.78129597,.64737647])
    assert choose([far,near],weights,float('inf')) is far
    assert choose([far,near],weights,165.521) is near
    assert choose([far,near],weights,1.) is None
