"""Record every marker reading the detector publishes, on the judge's clock.

The detector publishes provisional readings on /mission/marker_candidates and
validated readings on /mission/marker_observations; both carry the camera
exposure stamp. The judge only listens (the mission node already subscribes to
both topics' publisher, so the detector's work does not change). Times are
exposure stamp minus the mission start, the same base as registered_event_s.

detections.jsonl gets one line per reading while the trial runs, so the board
can show markers seen but not yet registered. The report summarises the first
sighting of every ID and which seen IDs were never registered.
"""
import json
import math

TOPICS = {'candidate': '/mission/marker_candidates', 'accepted': '/mission/marker_observations'}


class DetectionLog:
    def __init__(self, path):
        self.path = path
        self.stream = None
        self.first = {}
        self.counts = {}
        self.rejected = 0

    def record(self, kind, data, start_sim, closed):
        """Append one reading. Readings before the start or after the close are ignored."""
        if start_sim is None or closed:
            return None
        try:
            ident = int(data['id'])
            elapsed = float(data['stamp'])-start_sim
        except (KeyError, TypeError, ValueError):
            self.rejected += 1
            return None
        if not math.isfinite(elapsed) or elapsed < 0.:
            return None
        edge = data.get('min_edge_pixels')
        point = data.get('map_point')
        row = dict(t=round(elapsed, 3), kind=kind, id=ident,
                   min_edge_px=round(float(edge), 1) if isinstance(edge, (int, float)) else None,
                   map_point=[round(float(v), 3) for v in point[:2]] if isinstance(point, list) and len(point) >= 2 else None)
        if self.stream is None:
            self.stream = self.path.open('a', buffering=1)
        self.stream.write(json.dumps(row)+'\n')
        key = (ident, kind)
        self.first[key] = min(elapsed, self.first.get(key, elapsed))
        self.counts[key] = self.counts.get(key, 0)+1
        return row

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def report(self, registered, expected):
        """First sighting per ID, counts, and the IDs seen but never registered."""
        ids = sorted({ident for ident, _ in self.first})
        first = {str(i): round(min(v for (j, _), v in self.first.items() if j == i), 3) for i in ids}
        by_kind = {kind: {str(i): round(self.first[(i, kind)], 3) for i in ids if (i, kind) in self.first}
                   for kind in TOPICS}
        counts = {kind: {str(i): self.counts[(i, kind)] for i in ids if (i, kind) in self.counts} for kind in TOPICS}
        registered = {int(i) for i in registered}
        return dict(first_detected_s=first, first_detected_by_kind_s=by_kind, detection_counts=counts,
                    detected_not_registered=[i for i in ids if i not in registered],
                    detected_unexpected_ids=[i for i in ids if i not in set(expected)],
                    detection_messages_rejected=self.rejected)
