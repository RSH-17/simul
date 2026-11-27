"""Two small schedulers added to the marker-events baseline.

MidWallSlot   guarantees one bounded wall pass in the middle of the mission
              when no wall phase has happened yet (baseline: 6/10 failures
              never entered the wall phase).
StuckEscape   detects repeated instant planner failures from the same spot
              (baseline runs 9 and 15: a phantom SLAM cell inside the robot
              footprint) and asks the mission for the existing scan-checked
              backup, or an in-place turn.
Neither reads ground truth; both only use the elapsed time, the number of
registered IDs, the robot's own pose and Nav2 results.
"""
import math


class MidWallSlot:
    def __init__(self, start_fraction=.30, max_seconds=90., max_views=6):
        self.start_fraction = start_fraction
        self.max_seconds = max_seconds
        self.max_views = max_views
        self.started = None
        self.done = False
        self.ended_reason = None
        self.views = 0
        self.earlier_wall = False

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
        return dict(started=self.started, done=self.done, ended=self.ended_reason, views=self.views)


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
