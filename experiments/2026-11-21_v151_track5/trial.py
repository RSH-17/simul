"""Keep v1.5.1 navigation parameters unchanged on the large track."""
import sys
from pathlib import Path
BASE=Path.home()/'camera-aware-tools/experiments/2026-11-21_camera_aware_policy'
sys.path.insert(0,str(BASE))
from strict_trial import StrictTrial, main

class TrackTrial(StrictTrial):
    def __init__(self,*args):
        super().__init__(*args)
        # Legacy track recorder would otherwise change global costmap to 4Hz.
        self.map_rate_client=None

if __name__=='__main__':main(TrackTrial,BASE/'camera.launch.py')
