"""Independent scoring against the required task outcome.

Every requirement is its own check with three values: True, False, or None
(not measured by this backend). An episode is a strict success only when every
check is True. A missing measurement is never counted as a pass.
"""
import math

CHECKS = ('correct_ids', 'approached', 'positions', 'collision_free', 'in_time')


def nearest_distances(trajectory, targets, until_s):
    """Closest trajectory point to each target, up to ``until_s``."""
    out = {}
    for t in targets:
        best = math.inf
        for x, y, stamp in trajectory:
            if stamp <= until_s:
                best = min(best, math.hypot(x - t.x, y - t.y))
        out[t.id] = best
    return out


def evaluate(record, scenario, criteria, marker_count):
    truth = {t.id for t in scenario.targets}
    checks = {}

    if not truth or record.reported_ids is None:
        checks['correct_ids'] = None
    else:
        reported = [int(i) for i in record.reported_ids]
        checks['correct_ids'] = set(reported) == truth and len(reported) == len(truth) == marker_count

    approach = dict(record.approach_m) if record.approach_m is not None else None
    if approach is None and record.trajectory is not None and truth:
        approach = nearest_distances(record.trajectory, scenario.targets, record.duration_s)
    approach = {int(k): float(v) for k, v in approach.items()} if approach is not None else None
    if not truth:
        checks['approached'] = None
    elif record.approach_witnessed is not None:
        checks['approached'] = truth <= {int(i) for i in record.approach_witnessed}
    elif approach is not None:
        checks['approached'] = all(approach.get(i, math.inf) <= criteria['approach_m'] for i in truth)
    else:
        checks['approached'] = None

    if not truth or record.position_error_m is None:
        checks['positions'] = None
    else:
        errors = {int(k): float(v) for k, v in record.position_error_m.items()}
        checks['positions'] = (truth <= errors.keys()
                               and max(errors[i] for i in truth) <= criteria['position_tolerance_m'])

    checks['collision_free'] = None if record.collisions is None else record.collisions <= criteria['max_collisions']
    if record.completed is None or record.seconds is None:
        checks['in_time'] = None
    else:
        checks['in_time'] = bool(record.completed) and record.seconds <= record.duration_s
    for name, value in record.extra_checks.items():
        checks[name] = None if value is None else bool(value)

    values = list(checks.values())
    status = 'failure' if False in values else 'unverified' if None in values else 'success'
    measured = [v for v in values if v is not None]
    return dict(status=status, checks=checks,
                measured_success=bool(measured) and all(measured),
                approach_m=approach)


def summarize(rows, group_by=None):
    """Aggregate evaluated rows: [{'record': {...}, 'verdict': {...}}, ...]."""
    def block(items):
        n = len(items)
        statuses = {s: sum(r['verdict']['status'] == s for r in items) for s in ('success', 'failure', 'unverified')}
        names = sorted({c for r in items for c in r['verdict']['checks']})
        checks = {c: dict(passed=sum(r['verdict']['checks'].get(c) is True for r in items),
                          failed=sum(r['verdict']['checks'].get(c) is False for r in items),
                          not_measured=sum(r['verdict']['checks'].get(c) is None for r in items)) for c in names}
        done = [r['record']['seconds'] for r in items if r['verdict']['status'] == 'success']
        return dict(episodes=n, **statuses,
                    measured_success=sum(r['verdict']['measured_success'] for r in items),
                    checks=checks,
                    success_mean_seconds=sum(done) / len(done) if done else None)

    out = block(rows)
    if group_by:
        groups = {}
        for r in rows:
            groups.setdefault(str(group_by(r)), []).append(r)
        out['groups'] = {k: block(v) for k, v in sorted(groups.items())}
    return out
