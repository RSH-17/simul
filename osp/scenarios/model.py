"""Scenarios with an explicit information boundary.

A scenario holds three things with different readers:

* ``mission(...)`` -- what a policy may know (start pose, announced marker
  count, time limit). This is the only scenario data passed to a policy.
* ``targets`` -- ground-truth marker IDs and positions, for the evaluator only.
* ``world()`` -- the full scene description, for the simulator backend only.
"""
import copy
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Optional

import yaml


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Mission:
    """Everything a policy is allowed to know about a scenario."""
    scenario_id: str
    marker_count: int
    duration_s: float
    start: Optional[Pose] = None


@dataclass(frozen=True)
class Target:
    id: int
    x: float
    y: float


@dataclass(frozen=True)
class Scenario:
    id: str
    kind: str                      # 'surrogate_yaml' | 'team_sim_seed' | 'team_sim_scene'
    source: str
    targets: tuple = ()            # evaluator only; empty when truth stays with an external judge
    start: Optional[Pose] = None
    _world: Mapping = field(default_factory=dict, repr=False, compare=False)

    def mission(self, marker_count: int, duration_s: float) -> Mission:
        return Mission(self.id, int(marker_count), float(duration_s), self.start)

    def world(self) -> dict:
        """Full scene for a simulator backend. Never hand this to a policy."""
        return copy.deepcopy(dict(self._world))


def _targets(raw):
    return tuple(Target(int(t['id']), float(t['x']), float(t['y'])) for t in raw)


def load_surrogate(path) -> Scenario:
    """A legacy 2-D surrogate scenario YAML (``legacy/scenarios*/``)."""
    path = Path(path)
    cfg = yaml.safe_load(path.read_text())
    start = Pose(*map(float, cfg['start'][:3]))
    return Scenario(str(cfg['seed']), 'surrogate_yaml', str(path), _targets(cfg['targets']), start, cfg)


def team_sim_seed(seed: int) -> Scenario:
    """A team_sim Gazebo exam layout. Its truth is held by the team_sim judge."""
    return Scenario(str(seed), 'team_sim_seed', f'team_sim:{seed}', (), None, {'seed': int(seed)})


def load_team_scene(path) -> Scenario:
    """The ``scene.json`` a completed team_sim trial recorded (evaluator truth)."""
    path = Path(path)
    scene = json.loads(path.read_text())
    spawn = scene.get('robot_start_expected')
    start = Pose(float(spawn['x']), float(spawn['y']), float(spawn['yaw'])) if spawn else None
    return Scenario(str(scene['seed']), 'team_sim_scene', str(path), _targets(scene['targets']), start, scene)
