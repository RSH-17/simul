"""Read-only bridge to the archived ``legacy/src/osp_mission`` package."""
import sys
from pathlib import Path

from osp.core.paths import LEGACY

LEGACY_PACKAGE = LEGACY / 'src/osp_mission'


def mission_package():
    """Import legacy ``osp_mission`` and verify it comes from ``legacy/src``.

    A sourced ROS workspace can put another ``osp_mission`` first on the path;
    silently running that copy would make results incomparable, so refuse it.
    """
    if str(LEGACY_PACKAGE) not in sys.path:
        sys.path.insert(0, str(LEGACY_PACKAGE))
    import osp_mission
    origin = Path(osp_mission.__file__).resolve()
    if not origin.is_relative_to(LEGACY_PACKAGE.resolve()):
        raise RuntimeError(f'osp_mission was imported from {origin}, expected {LEGACY_PACKAGE}. '
                           'Run without a sourced osp_mission workspace.')
    return osp_mission
