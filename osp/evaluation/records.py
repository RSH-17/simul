"""What a backend reports for one episode. Truth-based scoring is the evaluator's job."""
from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass
class EpisodeRecord:
    scenario_id: str
    backend: str
    policy: str
    duration_s: float
    seconds: Optional[float] = None              # mission time used (simulated seconds)
    completed: Optional[bool] = None             # the mission ended by finishing, not by the deadline
    collisions: Optional[int] = None
    reported_ids: Optional[list] = None          # marker IDs the policy reported
    # Independent approach evidence. A backend gives either per-marker nearest
    # distances, a trajectory the evaluator measures, or both. `approach_witnessed`
    # is the set an external judge certified (e.g. within range after registration).
    approach_m: Optional[dict] = None
    approach_witnessed: Optional[list] = None
    trajectory: Optional[list] = field(default=None, repr=False)   # [(x, y, t), ...] in world frame
    position_error_m: Optional[dict] = None      # map position error per marker ID
    extra_checks: dict = field(default_factory=dict)   # backend-specific required checks
    backend_success: Optional[bool] = None       # the backend's own flag; reference only
    metrics: dict = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)

    def to_json(self, include_trajectory=False):
        data = asdict(self)
        if not include_trajectory:
            data.pop('trajectory')
        return data
