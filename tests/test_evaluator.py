from osp.evaluator import evaluate, summarize
from osp.records import EpisodeRecord
from osp.scenario import Scenario, Target

CRITERIA = dict(approach_m=1.5, position_tolerance_m=0.25, max_collisions=0)
SCENE = Scenario('s', 'surrogate_yaml', 'x', (Target(1, 0, 0), Target(2, 5, 0), Target(3, 0, 5)))


def record(**kw):
    base = dict(scenario_id='s', backend='t', policy='p', duration_s=600, seconds=300, completed=True,
                collisions=0, reported_ids=[1, 2, 3], approach_m={1: 1.0, 2: 1.4, 3: 0.5},
                position_error_m={1: .1, 2: .2, 3: .05})
    base.update(kw)
    return EpisodeRecord(**base)


def test_all_checks_pass():
    v = evaluate(record(), SCENE, CRITERIA, 3)
    assert v['status'] == 'success' and all(v['checks'].values())


def test_each_requirement_fails_on_its_own():
    cases = {'correct_ids': dict(reported_ids=[1, 2, 9]), 'approached': dict(approach_m={1: 1, 2: 1.6, 3: 1}),
             'positions': dict(position_error_m={1: .1, 2: .3, 3: .1}), 'collision_free': dict(collisions=1),
             'in_time': dict(completed=False, seconds=600)}
    for check, change in cases.items():
        v = evaluate(record(**change), SCENE, CRITERIA, 3)
        assert v['status'] == 'failure' and v['checks'][check] is False, check
        assert [c for c, ok in v['checks'].items() if ok is False] == [check]


def test_extra_ids_fail():
    assert evaluate(record(reported_ids=[1, 2, 3, 4]), SCENE, CRITERIA, 3)['checks']['correct_ids'] is False


def test_unmeasured_is_never_a_pass():
    v = evaluate(record(position_error_m=None), SCENE, CRITERIA, 3)
    assert v['status'] == 'unverified' and v['checks']['positions'] is None and v['measured_success']


def test_trajectory_distances_respect_the_deadline():
    traj = [(0, 0, 10), (5, 1, 20), (0, 4, 700)]          # target 3 reached only after the deadline
    v = evaluate(record(approach_m=None, trajectory=traj), SCENE, CRITERIA, 3)
    assert v['approach_m'][2] == 1.0 and v['approach_m'][3] == 5.0 and v['checks']['approached'] is False


def test_witnessed_approach_takes_precedence_over_distances():
    v = evaluate(record(approach_witnessed=[1, 2]), SCENE, CRITERIA, 3)
    assert v['checks']['approached'] is False


def test_summary_counts_and_groups():
    rows = [dict(record=record().to_json(), verdict=evaluate(record(), SCENE, CRITERIA, 3)),
            dict(record=record(collisions=2).to_json(), verdict=evaluate(record(collisions=2), SCENE, CRITERIA, 3))]
    s = summarize(rows, lambda r: r['record']['collisions'])
    assert (s['episodes'], s['success'], s['failure']) == (2, 1, 1)
    assert s['checks']['collision_free'] == dict(passed=1, failed=1, not_measured=0)
    assert set(s['groups']) == {'0', '2'}
