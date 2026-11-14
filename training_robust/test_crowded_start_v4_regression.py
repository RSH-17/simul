"""An observed exit cannot conceal missing actuator or transport evidence."""
import csv
import json

import pytest

from run_crowded_start_regression_v4 import collect


@pytest.mark.parametrize('missing', [None, 'drive_path_gate', 'simulation_drive_contract_gate',
                                    'navigation_transport_gate'])
def test_exit_requires_drive_and_transport_evidence(tmp_path, missing):
    episode = tmp_path / 'episodes/1'
    episode.mkdir(parents=True)
    (episode / 'evaluation.json').write_text(json.dumps(
        dict(reason='timeout', collisions=0, success=False)))
    with (episode / 'ground_truth_trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_seconds', 'world_x', 'world_y'])
        writer.writerow([5., 4.6, 5.])
        writer.writerow([30., 8.4, 5.])
    job = dict(output=str(tmp_path), scenario={'seed': 1}, configuration_gate=True,
               platform_gate=True, transfer_gate=True, floor_sensor_gate=True,
               drive_path_gate=True, simulation_drive_contract_gate=True,
               navigation_transport_gate=True)
    if missing:
        job.pop(missing)
    collect(job)
    assert job['outcome']['escaped'] is (missing is None)
    assert job['outcome']['valid_input'] is (missing is None)
    assert job['outcome']['full_task_success'] is False
