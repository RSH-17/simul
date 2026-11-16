from concurrent.futures import Future
from types import SimpleNamespace
import copy

import numpy as np
import pytest
from osp_mission.rolling_goal import RollingGoal,RollingGoalRuntime


def goal(x=3.,gain=.4,kind='surface',yaw=0.):
    return dict(x=x,y=0.,yaw=yaw,distance=abs(x),scan_angle=1.,coverage_kind=kind,
        features=np.array([gain if kind=='surface' else 0.,gain if kind=='frontier' else 0.,abs(x),0.,0.]))


def policy():
    p=RollingGoal();p.selected(goal(),0.);return p


def test_minimum_hold_reassessment_period_and_cooldown_are_simulation_time():
    p=policy();assert not p.due(11.9) and p.due(12.)
    p.last_check=12.;assert not p.due(19.9) and p.due(20.)
    p.acknowledged((3.,0.),20.);p.selected(goal(2.),21.)
    assert not p.due(39.9) and p.due(40.)


def test_small_gain_does_not_interrupt_but_substantial_gain_can():
    p=policy();current=goal();small=goal(1.5,.25);better=goal(1.5,1.5)
    assert p.consider([current,small],(3.,0.,0.),(0.,0.),np.ones(5),12.,500.) is None
    choice=p.consider([current,better],(3.,0.,0.),(0.,0.),np.ones(5),20.,500.)
    assert choice['reason']=='substantially_better_destination'
    assert choice['new_utility_after_switch_cost']>1.4*choice['old_utility']


@pytest.mark.parametrize('alternative',[goal(3.2,5.),goal(1.5,5.)])
def test_same_destination_or_near_arrival_does_not_oscillate(alternative):
    p=policy()
    robot=(0.,0.) if alternative['x']>3 else (2.5,0.)
    assert p.consider([goal(),alternative],(3.,0.,0.),robot,np.ones(5),12.,500.) is None


def test_missing_current_sample_is_not_proof_of_blockage():
    p=policy();choices=[goal(1.5,1.5)]
    assert p.consider(choices,(3.,0.,0.),(0.,0.),np.ones(5),12.,500.) is None
    assert p.consider(choices,(3.,0.,0.),(0.,0.),np.ones(5),20.,500.,blocked=True)['reason']=='destination_blocked'


def test_recent_abandoned_odom_anchor_prevents_return_and_then_expires():
    p=policy();old=goal();best=goal(1.5,1.5);best['_rolling_anchor']=(10.,-2.)
    p.acknowledged((10.1,-2.),0.)
    assert p.consider([old,best],(3.,0.,0.),(0.,0.),np.ones(5),30.,500.) is None
    assert p.consider([old,best],(3.,0.,0.),(0.,0.),np.ones(5),46.,500.) is not None


def test_maximum_switches_and_deadline_bound_reselection():
    p=policy()
    for i in range(6):p.acknowledged((i,0.),i*30.)
    assert not p.due(300.)
    assert policy().consider([goal(),goal(1.5,2.)],(3.,0.,0.),(0.,0.),np.ones(5),12.,4.) is None


@pytest.mark.parametrize('protected',[dict(start_escape=True),dict(stationary=True),dict(focus_id=0),dict(approach_id=42)])
def test_protected_navigation_never_starts_replanning(protected):
    owner=SimpleNamespace(now=lambda:20.,edge_mode=True,kind='nav',pending=False,goal_handle=object(),cancel_reason='')
    runtime=RollingGoalRuntime(owner);runtime.policy.selected(dict(goal(),**protected),0.)
    assert not runtime.tick() and runtime.checks==0


def prepared_owner():
    owner=SimpleNamespace(clock=20.,kind='nav',state='running',goal_handle=None,
        resume_action=None,scan_after=True,plan_after=0.,excluded=[],destination=(3.,0.),
        exploration_memory=SimpleNamespace(pending=object()),clears=[])
    owner.now=lambda:owner.clock
    owner.marker_look=SimpleNamespace(clear=owner.clears.append)
    owner.face_look=SimpleNamespace(clear=owner.clears.append)
    runtime=RollingGoalRuntime(owner);runtime.event=lambda *a,**kw:None
    runtime.pending=dict(old_anchor=(3.,0.));owner.rolling_goal=runtime
    return owner,runtime


def test_actual_mission_cancel_ack_replans_without_blacklisting_or_coverage_credit():
    from osp_mission.mission import Mission
    owner,runtime=prepared_owner();owner.cancel_reason='rolling_goal_reselect'
    future=Future();future.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,future)
    assert owner.exploration_memory.pending is None and owner.excluded==[]
    assert not owner.scan_after and owner.plan_after==20.25 and runtime.policy.switches==1
    assert runtime.pending is None and len(owner.clears)==2


@pytest.mark.parametrize('status,reason',[(4,'rolling_goal_reselect'),(5,'scan_stale'),(6,'')])
def test_completion_races_and_safety_cancels_remain_with_original_handler(status,reason):
    owner,runtime=prepared_owner();pending=owner.exploration_memory.pending
    assert not runtime.completed(status,'nav',reason)
    assert runtime.pending is None and owner.exploration_memory.pending is pending
    assert runtime.policy.switches==0


def test_contact_abort_and_new_selection_discard_unacknowledged_proposal():
    owner,runtime=prepared_owner();runtime.abort('bump_contact')
    assert runtime.pending is None and runtime.policy.switches==0
    runtime.pending=dict(old_anchor=(1.,2.));runtime.selected(goal())
    assert runtime.pending is None and runtime.policy.assigned_at==20.


def test_live_reassessment_uses_sensor_map_and_cannot_mutate_memory_or_send_early():
    from nav2_msgs.action import NavigateToPose
    class Memory:
        def __init__(self):self.pending=object();self.filtered=0
        def filter(self,choices,transform,stamp):self.filtered+=1;return choices
    owner=SimpleNamespace(clock=20.,now=lambda:20.,edge_mode=True,kind='nav',pending=False,
        goal_handle=object(),cancel_reason='',started=0.,weights=np.ones(5),inspection_camera=(221.,1.2),
        near_pass=SimpleNamespace(task=None),exploration_memory=Memory())
    owner.p=lambda k:600. if k=='duration' else {'map_frame':'map','base_frame':'base_link','odom_frame':'odom'}[k]
    q=SimpleNamespace(w=1.,x=0.,y=0.,z=0.)
    tf=SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.),rotation=q))
    owner.tf=SimpleNamespace(lookup_transform=lambda *a:tf)
    owner.map=SimpleNamespace(data=np.zeros((20,50)).ravel(),info=SimpleNamespace(
        height=20,width=50,resolution=.1,origin=SimpleNamespace(position=SimpleNamespace(x=-1.,y=-1.))))
    owner.known_dock_pose=lambda frame:None
    owner.coverage=SimpleNamespace(grid=lambda *a:np.zeros((4,20,50),dtype=bool))
    owner.policy_options=lambda *a,**kw:([goal(),goal(1.5,2.)],None)
    owner.active_goal=NavigateToPose.Goal();owner.active_goal.pose.pose.position.x=3.
    owner.active_goal.pose.pose.orientation.w=1.
    owner.canceled=[];owner.cancel_action=owner.canceled.append
    owner.get_logger=lambda:SimpleNamespace(debug=lambda msg:pytest.fail(msg))
    runtime=RollingGoalRuntime(owner);runtime.event=lambda *a,**kw:None
    runtime.policy.selected(goal(),0.)
    assert runtime.tick()
    assert owner.canceled==['rolling_goal_reselect'] and runtime.pending is not None
    assert owner.exploration_memory.filtered==0 and runtime.policy.switches==0
    assert not runtime.tick()  # Wait for cancellation acknowledgement.
