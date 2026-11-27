"""Named scenario sets, defined in ``configs/scenarios.yaml``."""
from pathlib import Path

import yaml

from osp import scenario as sc
from osp.paths import CONFIGS, resolve


def _definitions(path=None):
    path = Path(path) if path else CONFIGS / 'scenarios.yaml'
    return yaml.safe_load(path.read_text())['sets']


def names(path=None):
    return sorted(_definitions(path))


def describe(name, path=None):
    return _definitions(path)[name]


def load(name, path=None):
    """All scenarios of a named set, in a stable order."""
    sets = _definitions(path)
    if name not in sets:
        raise KeyError(f'Unknown scenario set {name!r}; known: {", ".join(sorted(sets))}')
    spec = sets[name]
    if spec['kind'] == 'surrogate_dir':
        directory = resolve(spec['path'])
        files = sorted(p for p in directory.glob('*.yaml') if p.name != 'manifest.yaml')
        if not files:
            raise FileNotFoundError(f'Scenario set {name!r} has no YAML files in {directory}')
        return [sc.load_surrogate(p) for p in files]
    if spec['kind'] == 'team_sim_seeds':
        return [sc.team_sim_seed(s) for s in spec['seeds']]
    raise ValueError(f'Scenario set {name!r} has unknown kind {spec["kind"]!r}')
