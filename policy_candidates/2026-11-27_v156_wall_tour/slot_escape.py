"""Small schedulers added to the marker-events baseline (v1.5.6 wall tour).

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
WallTour      v1.5.6: instead of one far segment, group every unviewed wall
              view far (>= 3 m) from visited positions into clusters, order
              them as a shortest open tour from the robot, keep the clusters
              that fit the slot budget, and visit them in that order.
              (v1.5.5 left 112605 at 0/3: both room ends were far and the
              slot went to one only; long-wall middles were skipped.)
None of these reads ground truth; both only use the elapsed time, the number of
registered IDs, the robot's own pose and Nav2 results.
"""
import itertools
import math

import numpy as np


class MidWallSlot:
    def __init__(self, start_fraction=.30, max_seconds=180., max_views=10):
        self.start_fraction = start_fraction
        self.max_seconds = max_seconds
        self.max_views = max_views
        self.started = None
        self.done = False
        self.ended_reason = None
        self.views = 0
        self.earlier_wall = False
        self.target = None
        self.tour = None

    def finish(self, reason):
        if not self.done:
            self._end(reason)

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

    def dispatched(self, phase, kind, goal=None, elapsed=None):
        if self.active:
            if kind == 'wall_surface':
                self.views += 1
                if self.tour is not None and goal is not None:
                    self.tour.dispatched(goal, elapsed)
        elif self.started is None and phase == 'wall':
            self.earlier_wall = True

    def report(self):
        return dict(started=self.started, done=self.done, ended=self.ended_reason, views=self.views,
                    target=None if self.target is None else [float(v) for v in self.target],
                    tour=None if self.tour is None else self.tour.report())


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


def _anchor(goal):
    return np.asarray(goal['surface_odom'], float)


def wall_clusters(items, visited, min_novelty=3., spacing=3.):
    """Group wall views far from visited positions; one seed per `spacing` metres.

    Seeds are taken most-novel first, so each cluster's target is its least
    visited point. Returns [(target, novelty, members)].
    """
    scored = []
    for _, goal in items:
        anchor = _anchor(goal)
        novelty = min((float(np.linalg.norm(anchor-p)) for p in visited), default=0.)
        if novelty >= min_novelty:
            scored.append((novelty, anchor))
    scored.sort(key=lambda pair: -pair[0])
    clusters = []
    for novelty, anchor in scored:
        for cluster in clusters:
            if np.linalg.norm(anchor-cluster[0]) < spacing:
                cluster[2] += 1
                break
        else:
            clusters.append([anchor, novelty, 1])
    return [tuple(c) for c in clusters]


def _path_length(start, points, order):
    total, here = 0., start
    for i in order:
        total += float(np.linalg.norm(points[i]-here))
        here = points[i]
    return total


def open_tour(start, points, exact_limit=7):
    """Shortest open path from start through every point (straight-line metres).

    Exact for up to `exact_limit` points, otherwise nearest-neighbour + 2-opt.
    """
    n = len(points)
    if n <= 1:
        return list(range(n))
    if n <= exact_limit:
        return list(min(itertools.permutations(range(n)), key=lambda o: _path_length(start, points, o)))
    order, left, here = [], set(range(n)), start
    while left:
        nxt = min(left, key=lambda i: float(np.linalg.norm(points[i]-here)))
        order.append(nxt)
        left.remove(nxt)
        here = points[nxt]
    improved = True
    while improved:
        improved = False
        for i in range(n-1):
            for j in range(i+1, n):
                trial = order[:i] + order[i:j+1][::-1] + order[j+1:]
                if _path_length(start, points, trial) < _path_length(start, points, order) - 1e-9:
                    order, improved = trial, True
    return order


class WallTour:
    """Visit every far, unviewed wall cluster that fits the slot, in tour order."""

    def __init__(self, targets, novelties, start, speed=.18, view_seconds=15., arrive=3., views_per_cluster=2,
                 leg_margin=30.):
        self.targets = [np.asarray(t, float) for t in targets]
        self.novelties = list(novelties)
        self.start = np.asarray(start, float)
        self.speed = speed
        self.arrive = arrive
        self.views_per_cluster = views_per_cluster
        self.view_seconds = view_seconds
        self.leg_margin = leg_margin
        self.index = 0
        self.leg_started = None
        self.leg_from = self.start
        self.views_here = 0
        self.skipped = []

    @classmethod
    def plan(cls, items, visited, here, budget, min_novelty=3., spacing=3., speed=.18, view_seconds=15.):
        clusters = wall_clusters(items, visited, min_novelty, spacing)
        if not clusters:
            return cls([], [], here, speed, view_seconds)
        points = [c[0] for c in clusters]
        order = open_tour(np.asarray(here, float), points)
        keep, spent, previous = [], 0., np.asarray(here, float)
        for i in order:
            leg = float(np.linalg.norm(points[i]-previous))/speed + view_seconds
            if keep and spent+leg > budget:
                break
            keep.append(i)
            spent += leg
            previous = points[i]
        return cls([points[i] for i in keep], [clusters[i][1] for i in keep], here, speed, view_seconds)

    @property
    def exhausted(self):
        return self.index >= len(self.targets)

    def _advance(self, reason, here, elapsed):
        self.skipped.append((self.index, reason))
        self.index += 1
        self.leg_started, self.leg_from, self.views_here = elapsed, np.asarray(here, float), 0

    def choose(self, items, here, elapsed):
        """The next tour view, or None once every cluster is done."""
        here = np.asarray(here, float)
        while not self.exhausted:
            target = self.targets[self.index]
            if self.leg_started is None:
                self.leg_started, self.leg_from = elapsed, here
            near = [item for item in items if np.linalg.norm(_anchor(item[1])-target) <= self.arrive]
            limit = float(np.linalg.norm(target-self.leg_from))/self.speed + self.leg_margin
            if not near:
                self._advance('seen', here, elapsed)
            elif self.views_here >= self.views_per_cluster:
                self._advance('views', here, elapsed)
            elif elapsed-self.leg_started > limit:
                self._advance('timeout', here, elapsed)
            else:
                if np.linalg.norm(here-target) > self.arrive:
                    goal = min(near, key=lambda item: float(np.linalg.norm(_anchor(item[1])-target)))[1]
                else:
                    goal = min(near, key=lambda item: item[0])[1]
                return dict(goal, wall_tour_index=self.index, wall_tour_size=len(self.targets),
                            far_wall_novelty_m=self.novelties[self.index])
        return None

    def dispatched(self, goal, elapsed):
        if goal.get('wall_tour_index') == self.index:
            self.views_here += 1

    def report(self):
        return dict(targets=[[float(v) for v in t] for t in self.targets],
                    novelty_m=[float(n) for n in self.novelties], index=self.index,
                    ended=[[int(i), r] for i, r in self.skipped])


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
