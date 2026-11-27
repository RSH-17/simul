"""Policy interfaces.

A policy is configured per backend (``policy.implementations.<backend>`` in its
YAML). Policies receive a ``Mission`` and sensor-derived observations only --
never a ``Scenario`` -- so ground truth cannot leak into decisions.
"""
from typing import Callable, Optional

import numpy as np

from osp.core.registry import POLICY_KINDS


class Policy:
    def __init__(self, name):
        self.name = name
        self.mission = None

    def prepare(self, mission):
        """Called once per episode with the policy-visible mission description."""
        self.mission = mission


class SurrogatePolicy(Policy):
    """A policy for the 2-D surrogate (``fast_sim`` backend).

    ``planner`` selects the surrogate's behaviour profile (scan sweep, coverage
    model), ``parameters`` are the policy weights. To try a new decision rule,
    subclass and set ``options`` / ``choose``; they replace the legacy
    candidate generator and selector and see only the observed map:

    * ``options(map, resolution, origin, robot_xy, yaw, visits, viewed, return_paths)``
      -> (candidates, predecessor_array)
    * ``choose(candidates, parameters)`` -> candidate dict with ``cell`` and ``yaw``, or None
    """
    options: Optional[Callable] = None
    choose: Optional[Callable] = None

    def __init__(self, name, planner, parameters):
        super().__init__(name)
        self.planner = planner
        self.parameters = np.asarray(parameters, dtype=float)


def build(cfg):
    """Instantiate the policy implementation for the configured backend."""
    spec = dict(cfg['policy']['implementations'][cfg['backend']['name']])
    return POLICY_KINDS.get(spec.pop('kind'))(cfg['policy']['name'], **spec)
