"""Record what the phased hybrid compared before each goal. Read-only: never changes a choice.

CenterWall.decide() keeps one trace per call in `self.trace`; PhasedHybrid writes it
to rule_trace.jsonl when the goal is dispatched. Each list is cut to the first
LIMIT entries in the rule's own order, and every helper swallows its own errors so
a tracing problem can only drop the trace, never the goal.
"""
import math

LIMIT = 10


def number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, 3) if math.isfinite(value) else None


def point(value):
    try:
        return [number(value[0]), number(value[1])]
    except (TypeError, IndexError, KeyError):
        return None


def interior_row(item, outside=None):
    """One unseen object face in BoxBatch.rank order.

    rank = (completed box, not same-box batch, not a fitted face, walking distance).
    """
    path, goal = item
    row = dict(x=number(goal.get('x')), y=number(goal.get('y')), path_m=number(path),
               straight_m=number(goal.get('distance')),
               completed_box=bool(goal.get('completed_box')), local_batch=bool(goal.get('local_batch')),
               fitted_face='fitted_face' in goal, previous_attempts=int(goal.get('previous_attempts') or 0),
               standoff_m=number(goal.get('standoff_m')), clearance_m=number(goal.get('clearance_m')),
               surface=point(goal.get('surface_map')), box=point(goal.get('batch_box')))
    if outside is not None:
        row['outside_crowded'] = any(other is item for other in outside)
    return row


def wall_row(item):
    """One wall view; lower score wins (path + standoff/shift penalties + 2 per attempt + continuity)."""
    score, goal = item
    return dict(x=number(goal.get('x')), y=number(goal.get('y')), score=number(score),
                straight_m=number(goal.get('distance')), previous_attempts=int(goal.get('previous_attempts') or 0),
                surface=point(goal.get('surface_map')))


def inputs(elapsed, duration, probe_only, candidates, deferred, fresh, boxes, crowded, outside):
    try:
        interior = candidates['interior']
        return dict(elapsed=number(elapsed), remaining_s=number(duration-elapsed), probe_only=bool(probe_only),
                    counts=dict(interior=len(interior), wall=len(candidates['wall']), deferred=len(deferred),
                                fresh=len(fresh), boxes=len(boxes)),
                    crowded_centers=[point(c) for c in crowded],
                    interior=[interior_row(item, outside) for item in interior[:LIMIT]],
                    wall=[wall_row(item) for item in sorted(candidates['wall'], key=lambda x: x[0])[:LIMIT]],
                    deferred=[interior_row(item) for item in deferred[:3]])
    except Exception as exc:  # tracing must never block a decision
        return dict(error=repr(exc))


def frontier_rank(choices, costmap, nav, veto):
    """discovery_frontier's filter and score, best first: gain/(1+.18*travel+max(0,2-travel)+novelty)."""
    rows = []
    for candidate in choices:
        gain = candidate['features'][1]
        if gain < .15 or candidate.get('stationary') or veto(candidate):
            continue
        cell = costmap.index(candidate['x'], candidate['y'])
        if not costmap.inside(*cell) or not nav[cell]:
            continue
        travel = candidate['distance']
        novelty = candidate['features'][4]
        rows.append(dict(x=number(candidate['x']), y=number(candidate['y']), gain=number(gain),
                         travel_m=number(travel), novelty=number(novelty),
                         score=number(gain/(1. + .18*travel + max(0., 2.-travel) + novelty))))
    rows.sort(key=lambda r: -(r['score'] or 0.))
    return rows[:LIMIT]


def gates(trace, fresh, frontier, wall_reserve, choices, costmap, nav, veto, outside=None):
    """Branch inputs after the crowded-box handling: the final fresh list and the frontier."""
    if not isinstance(trace, dict) or 'error' in trace:
        return
    try:
        trace['fresh'] = [interior_row(item, outside) for item in fresh[:LIMIT]]
        trace['frontier_chosen'] = None if frontier is None else point((frontier['x'], frontier['y']))
        trace['wall_reserve_s'] = number(wall_reserve)
        trace['frontiers'] = [] if fresh else frontier_rank(choices, costmap, nav, veto)
    except Exception as exc:
        trace['error'] = repr(exc)


def slot(trace, frontiers, report):
    if not isinstance(trace, dict) or 'error' in trace:
        return
    try:
        trace['slot_frontiers'] = len(frontiers)
        trace['slot'] = report
    except Exception as exc:
        trace['error'] = repr(exc)


def chosen_index(rows, goal):
    for i, row in enumerate(rows or []):
        if row.get('x') == number(goal.get('x')) and row.get('y') == number(goal.get('y')):
            return i
    return None


def record(trace, goal, reason, phase, elapsed):
    """The row written to rule_trace.jsonl for one dispatched goal."""
    row = dict(elapsed=number(elapsed), reason=reason, phase=phase, kind=goal.get('r32_kind', 'flow_or_marker'),
               goal=point((goal.get('x'), goal.get('y'))), trace=trace)
    if isinstance(trace, dict):
        for key in ('fresh', 'interior', 'wall', 'frontiers', 'deferred'):
            index = chosen_index(trace.get(key), goal)
            if index is not None:
                row['chosen_from'] = dict(list=key, rank=index+1)
                break
    return row
