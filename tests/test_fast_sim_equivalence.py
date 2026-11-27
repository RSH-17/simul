"""The new fast_sim backend must reproduce legacy/training_robust/evaluate.py exactly."""
import sys

import numpy as np
import pytest

from osp import catalog, config
from osp.legacy import mission_package
from osp.paths import LEGACY
from osp.policies.base import SurrogatePolicy, build
from osp.registry import BACKENDS, load_builtins

RESULT_KEYS = ('seed', 'difficulty', 'found', 'success', 'seconds', 'collisions', 'safety_stops',
               'distance', 'decisions', 'reward', 'coverage')


def legacy_task():
    mission_package()
    sys.path.insert(0, str(LEGACY / 'training_robust'))
    try:
        from evaluate import task
    finally:
        sys.path.remove(str(LEGACY / 'training_robust'))
    return task


def run_new(cfg, scenario, policy=None):
    load_builtins()
    backend = BACKENDS.get('fast_sim')(cfg['backend']['settings'])
    policy = policy or build(cfg)
    record, _ = backend.run_episode(scenario, policy, scenario.mission(3, cfg['mission']['duration_s']))
    return record, policy


@pytest.mark.parametrize('policy_name,set_name,count', [
    ('face_intent_v2', 'robust/test', 12),
    ('surface_endpoint', 'robust/validation', 6),
    ('mixed_v151', 'challenge/validated', 4),
])
def test_matches_legacy_evaluate(policy_name, set_name, count):
    task = legacy_task()
    cfg = config.load('fast_sim', policy_name)
    for scenario in catalog.load(set_name)[:count]:
        record, policy = run_new(cfg, scenario)
        old = task((scenario.world(), policy.planner, policy.parameters))
        assert {k: record.metrics[k] for k in RESULT_KEYS} == {k: old[k] for k in RESULT_KEYS}, scenario.id
        assert record.metadata['speed'] == old['speed'] and record.metadata['dropout'] == old['dropout']
        assert sorted(record.reported_ids) == sorted(set(t.id for t in scenario.targets)
                                                     - {m['id'] for m in old['missing']})


def test_custom_decision_hooks_replace_the_legacy_selector():
    cfg = config.load('fast_sim', 'face_intent_v2')
    scenario = catalog.load('robust/test')[0]
    baseline, legacy_policy = run_new(cfg, scenario)
    from osp_mission.edge_policy import choose, options
    calls = []

    class Counting(SurrogatePolicy):
        def choose(self, candidates, parameters):          # noqa: D401 - hook signature
            calls.append(len(candidates))
            return choose(candidates, parameters)

    policy = Counting('counting', legacy_policy.planner, legacy_policy.parameters)
    policy.options = options
    record, _ = run_new(cfg, scenario, policy)
    assert calls and record.metrics == baseline.metrics


def test_policy_cannot_see_the_scenario():
    cfg = config.load('fast_sim', 'face_intent_v2')
    scenario = catalog.load('robust/test')[0]
    seen = []

    class Spy(SurrogatePolicy):
        def prepare(self, mission):
            seen.append(mission)
            super().prepare(mission)

    base = build(cfg)
    run_new(cfg, scenario, Spy('spy', base.planner, base.parameters))
    assert seen == [scenario.mission(3, 600)]
    assert not np.isnan(base.parameters).any()
