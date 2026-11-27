"""Archived surrogate policies: a legacy ``policy.json`` (planner + weights)."""
import json

from osp.paths import resolve
from osp.policies.base import SurrogatePolicy
from osp.registry import POLICY_KINDS


@POLICY_KINDS.register('legacy_planner')
def legacy_planner(name, policy_file, planner=None):
    data = json.loads(resolve(policy_file).read_text())
    planner = planner or data.get('planner')
    if not planner:
        raise ValueError(f'{policy_file} names no planner; set implementations.fast_sim.planner')
    return SurrogatePolicy(name, planner, data['parameters'])
