"""Gazebo policies run by ``legacy/tools/team_sim`` from a prepared workspace."""
from osp.paths import resolve
from osp.policies.base import Policy
from osp.registry import POLICY_KINDS


class TeamSimPolicy(Policy):
    def __init__(self, name, team_policy, workspace):
        super().__init__(name)
        self.team_policy = team_policy          # name accepted by `team-sim.sh setup --policy`
        self.workspace = resolve(workspace)     # prepared colcon workspace


@POLICY_KINDS.register('team_sim')
def team_sim(name, team_policy, workspace):
    return TeamSimPolicy(name, team_policy, workspace)
