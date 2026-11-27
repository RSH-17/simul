from concurrent.futures import Future
from types import MethodType, SimpleNamespace

from nav2_msgs.action import NavigateToPose, Spin
import pytest

from osp_mission.mission import Mission
from osp_mission.navigation_budget import NavigationBudget
from osp_mission.near_pass_runtime import NearPassRuntime


def test_recorded_u_inspection_resumes_cannot_restart_navigation_allowance():
    budget=NavigationBudget();budget.begin(44.5)
    budget.pause(49.);assert budget.resume(52.5)
    budget.pause(99.5);assert budget.resume(102.5)
    assert not budget.expired(115.9)
    assert budget.expired(116.)
    assert budget.report(116.)['resumes']==2


def test_long_sensor_wait_is_excluded_but_prior_navigation_is_retained():
    budget=NavigationBudget();budget.begin(10.);budget.pause(30.)
    assert budget.resume(330.)
    assert budget.spent(340.)==30.
    budget.pause(340.);budget.pause(341.)
    assert budget.used_s==30.
    assert budget.resume(500.)
    assert budget.expired(535.)


def test_only_explicit_new_goal_gets_a_fresh_allowance():
    budget=NavigationBudget();assert not budget.resume(1.)
    budget.begin(10.);budget.pause(75.)
    assert not budget.resume(100.)
    budget.begin(100.)
    assert budget.spent(101.)==1. and not budget.expired(101.)


def test_resending_an_unacknowledged_active_goal_is_not_a_pause():
    budget=NavigationBudget();budget.begin(1.)
    with pytest.raises(RuntimeError,match='acknowledged'):
        budget.resume(2.)
    assert budget.spent(3.)==2.


@pytest.mark.parametrize('bad_time',[0.,float('nan'),float('inf')])
def test_clock_fault_cannot_replenish_remaining_allowance(bad_time):
    budget=NavigationBudget();budget.begin(10.);budget.pause(20.)
    assert not budget.resume(bad_time)
    assert budget.report(25.)['clock_fault']
    assert not budget.resume(26.)


class Handle:
    accepted=True
    def __init__(self):
        self.result=Future();self.cancels=0
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.cancels+=1


class Client:
    def __init__(self):self.handles=[]
    def send_goal_async(self,goal,**kwargs):
        handle=Handle();self.handles.append(handle)
        future=Future();future.set_result(handle);return future


def owner():
    o=SimpleNamespace(clock=10.,state='running',pending=False,goal_handle=None,
        cancel_reason='',action_generation=0,spin_progress=0.,sensor_interruptions=0,
        resume_action=None,scan_after=False,plan_after=0.,destination=(2.,3.),
        excluded=[],stops=0,targets={},collision_count=0,started=0.,
        spin_blocked_until=0.,recovery_pending=False,initial_scan=False,
        surface_mode=False,edge_mode=False,inspect=False)
    o.now=lambda:o.clock;o.p=lambda name:600.
    o.get_logger=lambda:SimpleNamespace(info=lambda s:None,warning=lambda s:None,error=lambda s:None)
    o.zero=lambda:setattr(o,'stops',o.stops+1)
    o.nav=Client();o.spin=Client();o.navigation_budget=NavigationBudget()
    o.memory_results=[]
    o.exploration_memory=SimpleNamespace(completed=lambda success,stamp:o.memory_results.append((success,stamp)))
    o.clears=[]
    o.marker_look=SimpleNamespace(clear=o.clears.append)
    o.face_look=SimpleNamespace(clear=o.clears.append)
    o.status=SimpleNamespace(publish=lambda message:None)
    o.ready=lambda:True
    o.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:NavigateToPose.Goal().pose.header.stamp))
    for name in ['send','accepted','completed','cancel_action','retire_expired_navigation','spin_feedback']:
        setattr(o,name,MethodType(getattr(Mission,name),o))
    return o


def test_actual_sensor_cancel_and_tick_resume_preserve_spent_time():
    o=owner();o.send(o.nav,NavigateToPose.Goal(),'nav')
    o.clock=30.;o.cancel_action('scan_stale')
    o.nav.handles[-1].result.set_result(SimpleNamespace(status=5))
    assert o.resume_action is not None and o.navigation_budget.used_s==20.
    o.clock=130.;Mission.tick(o)
    assert len(o.nav.handles)==2 and o.navigation_budget.spent(130.)==20.
    o.clock=175.;Mission.tick(o)
    assert o.cancel_reason=='navigation_budget_exhausted'
    assert o.nav.handles[-1].cancels==1
    o.nav.handles[-1].result.set_result(SimpleNamespace(status=5))
    assert o.memory_results==[(False,175.)]
    assert o.excluded==[(2.,3.)] and not o.scan_after


def test_expired_sensor_resume_sends_no_new_action_or_success_credit():
    o=owner();o.send(o.nav,NavigateToPose.Goal(),'nav')
    o.clock=76.;o.cancel_action('scan_stale')
    o.nav.handles[-1].result.set_result(SimpleNamespace(status=5))
    o.clock=200.;Mission.tick(o)
    assert len(o.nav.handles)==1 and not o.pending and o.goal_handle is None
    assert o.resume_action is None and o.memory_results==[(False,200.)]
    assert o.excluded==[(2.,3.)] and o.stops==1
    assert o.clears==['navigation_budget_exhausted']*2


def test_actual_inspection_cancel_turn_dwell_resume_preserves_target_budget():
    o=owner();o.marker_focus=SimpleNamespace(hints={});o.camera_diagnostics={'accepted':10}
    o.near_pass=NearPassRuntime(o);o.near_pass.event=lambda *a,**kw:None
    o.near_pass.camera_pose=lambda frame:(0.,0.,0.)
    o.send(o.nav,NavigateToPose.Goal(),'nav')
    o.clock=50.
    choice=dict(turn_rad=.7,odom_camera=[0.,0.,0.],odom_yaw=.7,estimated_sim_s=3.)
    o.near_pass.policy.reserve(choice,o.now())
    o.near_pass.task=dict(phase='cancel_nav',resume_goal=o.active_goal,desired_odom_yaw=.7,
        started=o.now(),cue_stamps={},confirmed_ids=set())
    o.cancel_action('near_pass_inspection')
    o.clock=50.5;o.nav.handles[-1].result.set_result(SimpleNamespace(status=5))
    assert o.navigation_budget.used_s==40.5 and not o.excluded
    assert o.near_pass.tick() and len(o.spin.handles)==1
    o.clock=53.;o.spin.handles[-1].result.set_result(SimpleNamespace(status=4))
    o.clock=54.;o.camera_diagnostics['accepted']+=4
    assert o.near_pass.tick() and o.resume_action is not None
    o.clock=54.5;Mission.tick(o)
    assert len(o.nav.handles)==2 and o.navigation_budget.spent(54.5)==40.5
    assert o.navigation_budget.report(54.5)['resumes']==1
    o.clock=79.;Mission.tick(o)
    assert o.cancel_reason=='navigation_budget_exhausted' and not o.excluded


def test_successful_navigation_keeps_normal_arrival_credit_and_new_goal_budget():
    o=owner();o.send(o.nav,NavigateToPose.Goal(),'nav')
    o.clock=30.;o.nav.handles[-1].result.set_result(SimpleNamespace(status=4))
    assert o.memory_results==[(True,30.)] and o.scan_after
    o.clock=35.;o.send(o.nav,NavigateToPose.Goal(),'nav')
    assert o.navigation_budget.spent(40.)==5.


def test_spin_action_retains_its_existing_timeout_instead_of_navigation_budget():
    o=owner();o.send(o.nav,NavigateToPose.Goal(),'nav')
    o.clock=30.;o.nav.handles[-1].result.set_result(SimpleNamespace(status=4))
    o.clock=40.;o.send(o.spin,Spin.Goal(),'spin')
    o.clock=106.;Mission.tick(o)
    assert o.cancel_reason=='action_timeout'
