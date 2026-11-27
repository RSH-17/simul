"""Small schedulers added to the marker-events baseline (v1.5.5 far-wall slot).

MidWallSlot   guarantees one bounded wall pass in the middle of the mission
              when no wall phase has happened yet (baseline: 6/10 failures
              never entered the wall phase).
StuckEscape   detects repeated instant planner failures from the same spot
              (baseline runs 9 and 15: a phantom SLAM cell inside the robot
              footprint) and asks the mission for the existing scan-checked
              backup, or an in-place turn.
far_wall /   v1.5.5: the slot's first wall view is the unviewed wall
toward_target  segment farthest from where the robot has been; until the robot
              is near it, further slot views stay at that segment.
None of these reads ground truth; both only use the elapsed time, the number of
registered IDs, the robot's own pose and Nav2 results.
"""
import math

import numpy as np


class MidWallSlot:
    def __init__(self, start_fraction=.30, max_seconds=120., max_views=6):
        self.start_fraction = start_fraction
        self.max_seconds = max_seconds
        self.max_views = max_views
        self.started = None
        self.done = False
        self.ended_reason = None
        self.views = 0
        self.earlier_wall = False
        self.target = None

    @property
    def active(self):
        return self.started is not None and not self.done

    def _end(self, reason):
        self.done, self.ended_reason = True, reason

    def update(self, elapsed, duration, found, has_wall):
        """Return True while the mid-mission wall pass should own the choice."""
        if self.done:
            return False
        if self.started is None:
            if self.earlier_wall:
                self._end('earlier_wall_phase')
                return False
            if elapsed < self.start_fraction*duration or found >= 3 or not has_wall:
                return False
            self.started = elapsed
        if elapsed-self.started >= self.max_seconds:
            self._end('time_limit')
        elif self.views >= self.max_views:
            self._end('view_limit')
        elif found >= 3:
            self._end('all_ids_registered')
        elif not has_wall:
            self._end('no_wall_views')
        return not self.done

    def dispatched(self, phase, kind):
        if self.active:
            if kind == 'wall_surface':
                self.views += 1
        elif self.started is None and phase == 'wall':
            self.earlier_wall = True

    def report(self):
        return dict(started=self.started, done=self.done, ended=self.ended_reason, views=self.views,
                    target=None if self.target is None else [float(v) for v in self.target])


def far_wall(items, visited, min_novelty=3., travel_weight=.15):
    """Pick the wall view whose surface lies farthest from the robot's own past positions.

    items are (score, goal) pairs from CenterWall; goal['surface_odom'] is the
    viewed wall point and goal['distance'] the straight-line travel. Returns
    None when every candidate is within min_novelty of a visited position.
    """
    best = None
    for _, goal in items:
        anchor = np.asarray(goal['surface_odom'], float)
        novelty = min((float(np.linalg.norm(anchor-p)) for p in visited), default=0.)
        if novelty < min_novelty:
            continue
        value = novelty - travel_weight*goal['distance']
        if best is None or value > best[0]:
            best = value, novelty, goal
    if best is None:
        return None
    return dict(best[2], far_wall_novelty_m=best[1])


def toward_target(items, target):
    """The wall view closest to the chosen far segment."""
    return min(items, key=lambda item: float(np.linalg.norm(np.asarray(item[1]['surface_odom'])-target)))[1]


class StuckEscape:
    def __init__(self, radius=.10, failures=3):
        self.radius = radius
        self.failures = failures
        self.anchor = None
        self.count = 0
        self.triggers = 0

    def quick_failure(self, xy):
        """Record an instant planner failure at xy; True when an escape is due."""
        if xy is None:
            return False
        if self.anchor is None or math.dist(self.anchor, xy) > self.radius:
            self.anchor, self.count = tuple(xy), 0
        self.count += 1
        if self.count < self.failures:
            return False
        self.count = 0
        self.triggers += 1
        return True

    def progress(self):
        self.anchor, self.count = None, 0
