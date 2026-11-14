"""Bound a navigation target across inspection and sensor-recovery resumes.

Only active navigation time is charged. Acknowledged cancellation pauses the
clock; inspection turns and sensor waits do not consume navigation allowance.
New target selection starts a fresh budget, resending the old target does not.
"""
import math


class NavigationBudget:
    def __init__(self, limit_s=65.):
        if not math.isfinite(limit_s) or limit_s <= 0:
            raise ValueError('Navigation time limit must be finite and positive')
        self.limit_s=float(limit_s)
        self.used_s=0.;self.active_since=None;self.last_stamp=None
        self.assigned=False;self.clock_fault=False;self.resumes=0

    def _valid_time(self, now):
        if not math.isfinite(now) or (self.last_stamp is not None and now < self.last_stamp):
            self.clock_fault=True
            return False
        self.last_stamp=now
        return True

    def begin(self, now):
        if not math.isfinite(now):
            raise ValueError('Invalid navigation start time')
        self.used_s=0.;self.active_since=now;self.last_stamp=now
        self.assigned=True;self.clock_fault=False;self.resumes=0

    def spent(self, now):
        if not self._valid_time(now) or self.clock_fault:
            return self.limit_s
        return self.used_s + (now-self.active_since if self.active_since is not None else 0.)

    def expired(self, now):
        return self.assigned and self.spent(now) >= self.limit_s

    def pause(self, now):
        if not self.assigned:
            return
        self.used_s=self.spent(now)
        self.active_since=None

    def resume(self, now):
        if not self.assigned or self.expired(now):
            return False
        if self.active_since is not None:
            raise RuntimeError('Navigation must be acknowledged as stopped before resuming')
        self.active_since=now;self.resumes+=1
        return True

    def report(self, now):
        return dict(limit_s=self.limit_s, active_s=self.spent(now) if self.assigned else 0.,
            assigned=self.assigned, active=self.active_since is not None,
            resumes=self.resumes, clock_fault=self.clock_fault,
            scope='Active navigation only; acknowledged inspection and sensor pauses excluded')
