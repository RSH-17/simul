"""Real action message schema, deterministic async success, rejection and late timeout."""
from types import SimpleNamespace
from unittest.mock import Mock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.time import Time
from osp_mission.path_review import PathReview

class Deferred:
    def add_done_callback(self,callback):self.callback=callback
    def result(self):return self.value
    def resolve(self,value):
        self.value=value
        self.callback(self)

def test_two_real_path_results_and_late_request_do_not_drive_before_review():
    node=Mock()
    node.p.return_value='map';node.now.return_value=10
    node.get_clock.return_value.now.return_value=Time(seconds=10)
    review=PathReview.__new__(PathReview)
    review.node=node;review.client=Mock();review.round=None;review.next_retry=0
    review.queries=0;review.failures={};review.rejected={};review.strikes={}
    futures=[Deferred() for _ in range(3)]
    review.client.send_goal_async.side_effect=futures
    goals=[dict(x=float(x),y=0.,yaw=0.) for x in (1,2,3)]
    review.start(goals,10,(0.,0.,0.))
    assert review.take(10) is None
    accepted=Mock(accepted=True);answer=Deferred();accepted.get_result_async.return_value=answer
    futures[0].resolve(accepted)
    msg=ComputePathToPose.Result()
    path=Path()
    for x,y in [(0.,0.),(0.,1.),(1.,0.)]:
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;path.poses.append(p)
    msg.path=path
    answer.resolve(SimpleNamespace(status=4,result=msg))
    futures[1].resolve(Mock(accepted=False))
    review.poll(16)
    assert review.round['done']
    late=Mock(accepted=True);futures[2].resolve(late);late.cancel_goal_async.assert_called_once()
    checked=review.take(16)['results']
    assert checked[0][1]>2.4 and checked[0][2] is None
    assert [r[2] for r in checked[1:]]==['planner_rejected','review_timeout']
    assert review.round is None and review.queries==3
