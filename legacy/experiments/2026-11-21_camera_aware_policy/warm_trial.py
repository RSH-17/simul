"""Final independent validation with a common 12s pre-mission sensor warmup."""
from pathlib import Path
from strict_trial import StrictTrial,main


class WarmTrial(StrictTrial):
    def prepare_start(self):
        return super().prepare_start() and self.sim is not None and self.sim>=12.


if __name__=='__main__':main(WarmTrial,Path(__file__).with_name('camera.launch.py'))
