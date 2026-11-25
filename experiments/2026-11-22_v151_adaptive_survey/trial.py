"""Unchanged balanced launch and passive judge; concurrent dashboard writes are atomic."""
import os
from pathlib import Path
import sys

experiments = Path(os.environ['SURVEY_EXPERIMENTS'])
sys.path.insert(0, str(experiments/'2026-11-21_camera_aware_policy'))
sys.path.insert(0, str(experiments/'2026-11-21_wall_transit_v152'))
from strict_trial import StrictTrial, main
from board_atomic import put


class SurveyTrial(StrictTrial):
    def __init__(self, *args):
        super().__init__(*args)
        self.map_rate_client = None
        self.board.put = put


if __name__ == '__main__':
    main(SurveyTrial, experiments/'2026-11-21_camera_aware_policy/camera.launch.py')
