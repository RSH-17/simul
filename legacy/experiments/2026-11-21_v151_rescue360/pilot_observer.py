"""Accelerated transition smoke test; separate from the 360-second policy setting."""
from pathlib import Path
import sys
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

HERE=Path(__file__).resolve().parent
import importlib.util
location=HERE.parent/'2026-11-21_wall_transit_v152/trial.py'
sys.path.insert(0,str(location.parent))
spec=importlib.util.spec_from_file_location('wall_trial_for_rescue',location)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
WallTrial,main=module.WallTrial,module.main

class PilotTrial(WallTrial):
    def __init__(self,*args):
        super().__init__(*args)
        self.configure=self.node.create_client(SetParametersAtomically,'/mission/set_parameters_atomically')
        self.setting=None
    def prepare_start(self):
        if self.setting is None:
            if self.configure.service_is_ready():
                request=SetParametersAtomically.Request()
                request.parameters=[Parameter('rescue_after_s',value=30.).to_parameter_msg()]
                self.setting=self.configure.call_async(request)
            return False
        if not self.setting.done():
            return False
        if not self.setting.result().result.successful:
            raise RuntimeError('Could not set pilot transition time')
        return super().prepare_start()

if __name__=='__main__':
    main(PilotTrial,HERE.parent/'2026-11-21_wall_transit_v152/wall.launch.py')
