"""Repository locations. Everything else is resolved relative to these."""
import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LEGACY = REPO / 'legacy'
CONFIGS = REPO / 'configs'


def runs_dir() -> Path:
    return Path(os.environ.get('OSP_RUNS', REPO / 'runs')).expanduser()


def team_sim_home() -> Path:
    return Path(os.environ.get('TEAM_SIM_HOME', Path.home() / '.local/share/simul-team')).expanduser()


def resolve(path) -> Path:
    """Expand ``~``; a relative path is relative to the repository root."""
    p = Path(path).expanduser()
    return p if p.is_absolute() else REPO / p
