"""Export finalized synthetic Gazebo videos and small reports for this campaign."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import tarfile

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('export_camera_summary',HERE/'summarize.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)
ROOTS=[('round1.json','camera-aware-policy-1121'),('round2.json','camera-refine-policy-1121'),
       ('camera_matrix.json','camera-matrix-policy-1121'),('final_validation.json','camera-final-policy-1121')]


def export(destination):
    destination.mkdir(parents=True,exist_ok=False);reports=destination/'reports';reports.mkdir()
    provenance=[]
    for filename,name in ROOTS:
        root=Path.home()/name
        assert (root/'complete.json').exists(),root
        result=summary.analyze(root);assert result['complete']
        (reports/filename).write_text(json.dumps(result,indent=2)+'\n')
        for run_name,row in zip(json.loads((root/'runs.json').read_text()),result['runs']):
            run=Path(run_name);audit=json.loads((run/'audit.json').read_text());metrics=json.loads((run/'metrics.json').read_text())
            assert run.name==row['run'] and metrics['world_sha256']==row['world_sha256']
            episode=destination/'training_robust/native90/camera-policy-1121'/run.name
            (episode/'video').mkdir(parents=True)
            shutil.copy2(run/'mission.mp4',episode/'video/mission.mp4')
            shutil.copy2(run/'audit.json',episode/'camera_audit.json')
            shutil.copy2(run/'metrics.json',episode/'camera_metrics.json')
            evaluation=dict(success=row['strict_saved_map_success'],reason=row['reason'],
                elapsed_seconds=row['elapsed_sim_s'],targets=len(row['registered_ids']),collisions=row['bumper_events'],
                protocol='3 correct IDs + independent <=1.5m + unchanged near RGB proof + saved map error <=0.25m + zero bumps',
                ground_truth_audit=dict(closest_approach_m=metrics['min_ground_truth_m']),
                provenance='Gazebo rendered RGB on localhost simulation domain 174; no physical robot images')
            (episode/'evaluation.json').write_text(json.dumps(evaluation,indent=2)+'\n')
            (episode/'video/recording.json').write_text(json.dumps(dict(frames=audit['video']['frames'],
                duration_seconds=audit['video']['duration_s']),indent=2)+'\n')
            provenance.append(dict(run=run.name,world_sha256=metrics['world_sha256'],
                generated_world=str(run/'world/osp_studio.sdf'),physical_images=False,
                camera=dict(width=row['camera']['width'],height=row['camera']['height'],fps=row['camera']['requested_fps']),
                strict_saved_map_success=row['strict_saved_map_success'],video_sha256=audit['video']['sha256']))
    (reports/'video_provenance.json').write_text(json.dumps(dict(source='Native Gazebo simulation, generated rooms and markers',
        physical_robot_used=False,domain=174,localhost_only=True,videos=provenance),indent=2)+'\n')
    with tarfile.open(destination.with_suffix('.tgz'),'w:gz') as archive:archive.add(destination,arcname=destination.name)
    print(json.dumps(dict(videos=len(provenance),archive=str(destination.with_suffix('.tgz')))))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('destination',type=Path);export(parser.parse_args().destination)
