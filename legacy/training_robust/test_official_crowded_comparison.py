"""Physical escape cannot qualify with missing official input evidence."""
import csv
import json

import pytest

from run_official_crowded_comparison import GATES, collect


@pytest.mark.parametrize('missing', [None, *GATES])
def test_escape_requires_every_gate(tmp_path, missing):
    episode = tmp_path / 'episodes/1'
    episode.mkdir(parents=True)
    (episode/'evaluation.json').write_text(json.dumps(dict(reason='timeout', collisions=0, success=False)))
    with (episode/'ground_truth_trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerows([['sim_seconds', 'world_x', 'world_y'], [5., 4.6, 5.], [30., 8.4, 5.]])
    job = dict(output=str(tmp_path), scenario={'seed': 1}, **dict.fromkeys(GATES, True))
    if missing:
        job.pop(missing)
    collect(job)
    assert job['outcome']['escaped'] is (missing is None)
    assert job['outcome']['full_task_success'] is False


@pytest.mark.parametrize('problem', ['contact', 'missing_truth', 'startup_failure', 'late_exit'])
def test_bad_outcome_does_not_qualify(tmp_path, problem):
    episode = tmp_path / 'episodes/1'
    episode.mkdir(parents=True)
    (episode/'evaluation.json').write_text(json.dumps(dict(
        reason='startup_failure' if problem == 'startup_failure' else 'timeout',
        collisions=1 if problem == 'contact' else 0, success=False)))
    if problem != 'missing_truth':
        with (episode/'ground_truth_trajectory.csv').open('w') as stream:
            csv.writer(stream).writerows([['sim_seconds', 'world_x', 'world_y'],
                [5., 4.6, 5.], [96. if problem == 'late_exit' else 30., 8.4, 5.]])
    job = dict(output=str(tmp_path), scenario={'seed': 1}, **dict.fromkeys(GATES, True))
    collect(job)
    assert not job['outcome']['escaped']
