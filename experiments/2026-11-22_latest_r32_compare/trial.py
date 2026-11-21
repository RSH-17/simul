"""Independent common judge; no target truth is supplied to either policy."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'2026-11-21_wall_transit_v152'))
import importlib.util
_spec=importlib.util.spec_from_file_location('wall_trial_parent',Path(__file__).resolve().parents[1]/'2026-11-21_wall_transit_v152/trial.py')
_parent=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_parent)
WallTrial,main=_parent.WallTrial,_parent.main

class LatestTrial(WallTrial):
    def on_status(self,msg):
        super().on_status(msg)
        self.landmarks={t['id']:(t['x'],t['y']) for t in self.status.get('targets',[]) if t['confirmed']}

    def sensor_proof_complete(self):
        # R32 does not implement Mixed's extra three-frame 1.3 m witness.
        # Common success uses independent registered-then-1.5 m evidence.
        return self.current==set(self.targets)

if __name__=='__main__':
    main(LatestTrial,Path(__file__).with_name('latest.launch.py'))
