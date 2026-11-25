"""The same passive strict judge and costmap rate as the completed baseline."""
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]/'2026-11-21_camera_aware_policy'
sys.path.insert(0, str(BASE))
from strict_trial import StrictTrial, main
from board_atomic import put


class WallTrial(StrictTrial):
    def __init__(self, *args):
        super().__init__(*args)
        self.map_rate_client = None
        # The live board can refresh the same card concurrently with this
        # observer. Keep this observer's temporary files private to each write.
        self.board.put = put


if __name__ == '__main__':
    main(WallTrial, Path(__file__).with_name('wall.launch.py'))
