import csv
import json

import pytest

from audit_pose_trace import audit


def test_audit_does_not_extrapolate_or_refit_odometry(tmp_path):
    (tmp_path/'evaluation.json').write_text(json.dumps(dict(success=False,
        ground_truth_audit=dict(world_to_map=[0.,0.,0.]))))
    with (tmp_path/'ground_truth_trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_seconds','world_x','world_y','world_yaw'])
        writer.writerows([[t,t,0.,0.] for t in [0.,.2,.4,.6,1.5]])
    with (tmp_path/'pose_trace.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['map_stamp','map_x','map_y','map_yaw','odom_stamp','odom_x','odom_y','odom_yaw'])
        for i,t in enumerate([.1,.3,.5,1.1,2.1]):
            writer.writerow([t,t,0.,0.,t,t+i*.1,0.,0.])
    result = audit(tmp_path)
    assert result['pose_errors']['map']['position_error_m']['samples'] == 3
    assert result['pose_errors']['map']['position_error_m']['maximum'] == pytest.approx(0.)
    assert result['pose_errors']['odom']['position_error_m']['maximum'] == pytest.approx(.2)
    assert result['full_task_success'] is False
