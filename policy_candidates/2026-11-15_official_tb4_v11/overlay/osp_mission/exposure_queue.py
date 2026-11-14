"""Bounded exposure-time queue; never substitutes a newer pose or timestamp."""
from collections import deque


class ExposureQueue:
    def __init__(self, capacity=8, maximum_age_ns=250_000_000):
        self.capacity = capacity
        self.maximum_age_ns = maximum_age_ns
        self.pending = deque()
        self.last_stamp = None
        self.last_now = None
        self.counts = dict(received=0, processed=0, expired=0, overflow=0,
                           duplicate_or_out_of_order=0, future=0, invalid_timestamp=0, clock_resets=0,
                           tf_wait_attempts=0)

    def observe_clock(self, now):
        if self.last_now is not None and now < self.last_now:
            self.pending.clear()
            self.last_stamp = None
            self.counts['clock_resets'] += 1
        self.last_now = now

    def push(self, stamp, payload, now):
        self.observe_clock(now)
        self.counts['received'] += 1
        # ROS Time(0) means "latest", not a known exposure timestamp.
        if stamp <= 0:
            self.counts['invalid_timestamp'] += 1
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.counts['duplicate_or_out_of_order'] += 1
            return
        if stamp-now > self.maximum_age_ns:
            self.counts['future'] += 1
            return
        self.last_stamp = stamp
        if now-stamp > self.maximum_age_ns:
            self.counts['expired'] += 1
            return
        if len(self.pending) >= self.capacity:
            self.pending.popleft()
            self.counts['overflow'] += 1
        self.pending.append((stamp, payload))

    def ready(self, now, can_transform):
        self.observe_clock(now)
        result = []
        while self.pending:
            stamp, payload = self.pending[0]
            if now-stamp > self.maximum_age_ns:
                self.pending.popleft()
                self.counts['expired'] += 1
                continue
            if not can_transform(stamp, payload):
                self.counts['tf_wait_attempts'] += 1
                break
            self.pending.popleft()
            self.counts['processed'] += 1
            result.append(payload)
        return result

    def report(self):
        return dict(self.counts, pending=len(self.pending), capacity=self.capacity,
                    maximum_age_ns=self.maximum_age_ns)
