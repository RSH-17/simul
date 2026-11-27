import importlib.util,json,os,sys,yaml
from pathlib import Path
helpers=Path(os.environ['CAMERA_BENCH_HELPERS'])
spec=importlib.util.spec_from_file_location('camera_board',helpers/'2026-11-19_corridor_box/compare_board.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
run=Path(sys.argv[1]);scene=json.loads((run/'scene.json').read_text())
layout=run/'run/simulation/demo_layout.json'
if layout.is_file():scene['partitions']+=json.loads(layout.read_text())['tables']
duration=float(yaml.safe_load((run/'scene.yaml').read_text())['duration'])
module.refresh(run,Path(os.environ['CAMERA_BENCH_CARDS'])/run.name,scene,{},origin=(0,0),duration=duration)
