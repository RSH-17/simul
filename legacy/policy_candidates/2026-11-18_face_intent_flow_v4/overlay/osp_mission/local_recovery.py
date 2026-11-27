"""Remember where local control failed across successive navigation goals."""
import math


class LocalRecovery:
    radius = .45
    sector = math.radians(45)
    sector_lifetime = 60.
    retry_delay = 15.

    def __init__(self):
        self.position = None
        self.position_time = -math.inf
        self.anchor = None
        self.failures = 0
        self.directions = []
        self.retreat_used = False
        self.retry_after = -math.inf
        self.next_rear_check = -math.inf
        self.last_reason = ''

    def observe(self, x, y, now):
        if not all(math.isfinite(v) for v in (x, y, now)):
            return
        self.position = (x, y)
        self.position_time = now
        if self.anchor is not None and math.dist(self.anchor, self.position) >= self.radius:
            self.anchor = None
            self.failures = 0
            self.directions.clear()
            self.retreat_used = False
            self.retry_after = -math.inf
            self.next_rear_check = -math.inf
            self.last_reason = ''

    def bearing(self, goal, now):
        if goal is None or goal[0] != 'odom' or now - self.position_time > 1.5:
            return None
        dx = goal[1] - self.position[0]
        dy = goal[2] - self.position[1]
        return math.atan2(dy, dx) if math.hypot(dx, dy) > .05 else None

    def failed(self, goal, reason, now):
        self.last_reason = reason
        if self.position is None or now - self.position_time > 1.5:
            self.retry_after = now + self.retry_delay
            return False
        if self.anchor is None:
            self.anchor = self.position
        self.failures += 1
        angle = self.bearing(goal, now)
        self.directions = [(a, expiry) for a, expiry in self.directions if expiry > now]
        if angle is not None:
            self.directions.append((angle, now + self.sector_lifetime))
        if self.failures >= 3:
            self.retry_after = now + self.retry_delay
        return self.failures >= 2 and not self.retreat_used

    def request_retreat(self, now):
        if self.anchor is None or self.retreat_used or now < self.next_rear_check:
            return False
        self.next_rear_check = now + self.retry_delay
        return True

    def excludes(self, goal, now):
        angle = self.bearing(goal, now)
        return angle is not None and any(
            now < expiry and abs(math.remainder(angle - failed, 2 * math.pi)) <= self.sector
            for failed, expiry in self.directions)

    def report(self):
        return dict(failures_here=self.failures, retreat_attempted=self.retreat_used,
                    last_local_failure=self.last_reason)


def mark_departures(goals, previous, shape, resolution, origin, robot):
    """Attach the first point roughly 40 cm along each exploration route."""
    if previous is None:
        return
    height, width = shape
    def center(index):
        row, col = divmod(index, width)
        return origin[0] + (col + .5) * resolution, origin[1] + (row + .5) * resolution

    cache = {}
    for goal in goals:
        cell = goal.get('cell')
        if cell is None:
            continue
        row, col = cell
        if not (0 <= row < height and 0 <= col < width):
            continue
        index = row * width + col
        trail = []
        while index >= 0 and index not in cache:
            trail.append(index)
            index = int(previous[index])
        departure = cache.get(index)
        for node in reversed(trail):
            point = center(node)
            if departure is None and math.dist(point, robot) >= .40:
                departure = point
            cache[node] = departure
        if departure is not None:
            goal['departure'] = departure
