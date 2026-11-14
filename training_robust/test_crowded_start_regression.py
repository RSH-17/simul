"""Negative controls for the independent crowded-start acceptance evidence."""
import csv
import json

import pytest

from run_crowded_start_regression import collect, scene


@pytest.mark.parametrize('exit_time,contacts,reason,platform,expected', [
    (25., 0, 'timeout', True, True),
    (125., 0, 'timeout', True, False),
    (25., 1, 'timeout', True, False),
    (25., 0, 'crashed', True, False),
    (25., 0, 'timeout', False, False),
    (None, 0, 'timeout', True, False),
])
def test_escape_requires_actual_timely_motion_and_valid_run(
        tmp_path, exit_time, contacts, reason, platform, expected):
    episode = tmp_path / 'episodes/1'
    episode.mkdir(parents=True)
    (episode / 'evaluation.json').write_text(json.dumps(
        dict(reason=reason, collisions=contacts, success=False)))
    with (episode / 'ground_truth_trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_seconds', 'world_x', 'world_y'])
        writer.writerow([5., 4.6, 5.])
        writer.writerow([5. + (exit_time or 25.), 8.4 if exit_time else 4.6, 5.])
    job = dict(output=str(tmp_path), scenario={'seed': 1}, configuration_gate=True,
               platform_gate=platform, transfer_gate=True, floor_sensor_gate=True)
    collect(job)
    assert job['outcome']['escaped'] is expected
    assert job['outcome']['full_task_success'] is False


@pytest.mark.parametrize('width,yaw', [(.55, 0.), (.60, 1.5707963267948966)])
def test_scene_has_body_clear_exit_and_hidden_obstacle_markers(width, yaw):
    config, evidence = scene(1196000000, width, yaw)
    assert evidence['exit_reachable'] and evidence['marker_approaches_reachable']
    assert evidence['start_clearance_m'] >= evidence['body_clearance_required_m']
    assert evidence['start_clearance_m'] < .32
    assert len(config['targets']) == 3
    assert all(target['mount'] == 'obstacle' for target in config['targets'])
