"""Layered run configuration.

Layers merge in this order, later winning:
``configs/base.yaml`` -> ``configs/backends/<backend>.yaml`` ->
``configs/policies/<policy>.yaml`` -> ``--set key.path=value`` overrides.
"""
import copy
from pathlib import Path

import yaml

from osp.core.paths import CONFIGS

# A config may never call itself production. Promotion is a separate,
# evidence-backed decision recorded outside this file.
POLICY_STATUSES = ('legacy_release', 'legacy_candidate', 'experimental')


class ConfigError(ValueError):
    pass


def deep_merge(base, update):
    out = copy.deepcopy(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def parse_override(text):
    """``a.b.c=value`` -> {'a': {'b': {'c': value}}}; the value is parsed as YAML."""
    if '=' not in text:
        raise ConfigError(f'Override {text!r} must look like key.path=value')
    key, raw = text.split('=', 1)
    parts = [p for p in key.strip().split('.') if p]
    if not parts:
        raise ConfigError(f'Override {text!r} has an empty key')
    node = value = yaml.safe_load(raw)
    for part in reversed(parts):
        node = {part: node}
    return node


def _read(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(data, dict):
        raise ConfigError(f'{path} must contain a mapping')
    return data


def base(root=CONFIGS):
    """Only the shared base layer (mission + criteria), e.g. to re-score imported trials."""
    return _read(Path(root) / 'base.yaml')


def available(kind, root=CONFIGS):
    return sorted(p.stem for p in (Path(root) / kind).glob('*.yaml'))


def load(backend, policy, overrides=(), root=CONFIGS):
    root = Path(root)
    layers = [root / 'base.yaml', root / 'backends' / f'{backend}.yaml', root / 'policies' / f'{policy}.yaml']
    for path, kind in zip(layers[1:], ('backend', 'policy')):
        if not path.is_file():
            known = ', '.join(available(path.parent.name, root))
            raise ConfigError(f'Unknown {kind} {path.stem!r}; known: {known}')
    merged = {}
    for path in layers:
        merged = deep_merge(merged, _read(path))
    for text in overrides:
        merged = deep_merge(merged, parse_override(text))
    merged['layers'] = [str(p.relative_to(root.parent)) if p.is_relative_to(root.parent) else str(p) for p in layers]
    merged['overrides'] = list(overrides)
    validate(merged)
    return merged


def validate(cfg):
    for section in ('mission', 'criteria', 'backend', 'policy'):
        if not isinstance(cfg.get(section), dict):
            raise ConfigError(f'Missing section {section!r}')
    mission, criteria, backend, policy = (cfg[k] for k in ('mission', 'criteria', 'backend', 'policy'))
    if not (isinstance(mission.get('marker_count'), int) and mission['marker_count'] > 0):
        raise ConfigError('mission.marker_count must be a positive integer')
    if not (isinstance(mission.get('duration_s'), (int, float)) and mission['duration_s'] > 0):
        raise ConfigError('mission.duration_s must be positive')
    for key in ('approach_m', 'position_tolerance_m'):
        if not (isinstance(criteria.get(key), (int, float)) and criteria[key] > 0):
            raise ConfigError(f'criteria.{key} must be positive')
    if not (isinstance(criteria.get('max_collisions'), int) and criteria['max_collisions'] >= 0):
        raise ConfigError('criteria.max_collisions must be a non-negative integer')
    if not backend.get('name'):
        raise ConfigError('backend.name is required')
    if policy.get('status') not in POLICY_STATUSES:
        raise ConfigError(f'policy.status must be one of {POLICY_STATUSES} (got {policy.get("status")!r})')
    impl = (policy.get('implementations') or {}).get(backend['name'])
    if not isinstance(impl, dict) or 'kind' not in impl:
        have = ', '.join(sorted(policy.get('implementations') or {})) or 'none'
        raise ConfigError(f'Policy {policy.get("name")!r} has no {backend["name"]!r} implementation (has: {have})')
    return cfg
