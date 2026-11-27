"""2-D surrogate backend: the legacy ``SearchEnv`` (not physics, not Gazebo).

Defaults reproduce ``legacy/training_robust/evaluate.py``: seed offset
123456, 960 px camera model, per-scenario ``sensor_stress`` overrides for
detection dropout and the speed limit.
"""
from osp.core.legacy import mission_package
from osp.policies.base import SurrogatePolicy
from osp.evaluation.records import EpisodeRecord
from osp.core.registry import BACKENDS


@BACKENDS.register('fast_sim')
class FastSimBackend:
    name = 'fast_sim'
    accepts = ('surrogate_yaml',)
    parallel_limit = None

    def __init__(self, settings):
        self.settings = dict(settings)

    def describe(self, scenario, policy, mission):
        return f'fast_sim {policy.name} scenario {scenario.id}'

    def run_episode(self, scenario, policy, mission, log_path=None):
        """Returns (record, truth_scenario); the surrogate scene is its own truth."""
        if not isinstance(policy, SurrogatePolicy):
            raise TypeError(f'fast_sim needs a surrogate policy, got {type(policy).__name__}')
        mission_package()
        from osp_mission.fast_sim import SearchEnv
        s = self.settings
        cfg = scenario.world()
        stress = cfg.get('sensor_stress', {})
        env = SearchEnv(cfg, seed=cfg['seed'] + s['seed_offset'], planner=policy.planner,
                        camera_width=s['camera_width'],
                        detection_dropout=stress.get('detection_dropout', s['detection_dropout']),
                        camera_hz=s['camera_hz'], inspection_frames=s['inspection_frames'],
                        **(s.get('env') or {}))
        env.speed = min(env.speed, stress.get('speed_limit', s['speed_limit']))
        if policy.options is not None:
            env.options = policy.options
        if policy.choose is not None:
            env.choose = policy.choose
        policy.prepare(mission)
        result = env.run(policy.parameters, deadline=mission.duration_s)
        record = EpisodeRecord(
            scenario_id=scenario.id, backend=self.name, policy=policy.name, duration_s=mission.duration_s,
            seconds=result['seconds'], completed=len(env.found) >= mission.marker_count, collisions=result['collisions'], reported_ids=sorted(env.found),
            trajectory=[(x, y, t) for x, y, _yaw, t in env.trace],
            backend_success=result['success'], metrics=result,
            metadata=dict(planner=policy.planner, speed=env.speed, dropout=env.detection_dropout))
        return record, scenario
