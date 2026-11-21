"""Use the original passive strict judge with a relocatable launch."""
import importlib.util
import os
import sys
from pathlib import Path
repo=Path(__file__).resolve().parents[2]
os.environ.setdefault('CAMERA_BENCH_HELPERS',str(Path(__file__).resolve().parent/'helpers'))
p=repo/'experiments/2026-11-21_wall_transit_v152/trial.py'
sys.path.insert(0,str(p.parent))
spec=importlib.util.spec_from_file_location('team_wall_trial',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
if __name__=='__main__':m.main(m.WallTrial,Path(__file__).with_name('portable.launch.py'))
