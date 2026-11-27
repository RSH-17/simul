"""Policy kinds. Importing this package registers the built-in kinds."""
from osp.policies import legacy_planner, team_sim  # noqa: F401
from osp.policies.base import Policy, SurrogatePolicy, build  # noqa: F401
