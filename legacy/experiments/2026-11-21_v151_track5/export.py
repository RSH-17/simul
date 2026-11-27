"""Export completed five-track results and synthetic videos, never raw ROS bags."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tarfile

BASE=Path.home()/'camera-aware-tools/experiments/2026-11-21_camera_aware_policy'
spec=importlib.util.spec_from_file_location('export_track_summary',BASE/'summarize.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)

def export(root,destination):
    assert (root/'complete.json').exists(),'Wait for all five native trials'
    report=summary.analyze(root)
    assert report['complete'] and len(report['runs'])==5
    assert len({r['world_sha256'] for r in report['runs']})==5
    for r in report['runs']:r['split']='large_track_validation'
    for g in report['groups']:g['split']='large_track_validation'
    report['notes'][0]='Five different predetermined large-track layouts; one frozen policy, no tuning or inter-policy matched comparison.'
    destination.mkdir(exist_ok=False)
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for name in ('protocol.json','runtime_parameters.json'):shutil.copy2(root/name,destination/name)
    provenance=[]
    for name,row in zip(json.loads((root/'runs.json').read_text()),report['runs']):
        run=Path(name);assert run.name==row['run']
        metrics=json.loads((run/'metrics.json').read_text());audit=json.loads((run/'audit.json').read_text())
        scene=json.loads((run/'scene.json').read_text())
        row['rendered_seed']=scene['seed']
        episode=destination/'training_robust/native90/v151-track-five1121'/run.name
        (episode/'video').mkdir(parents=True)
        shutil.copy2(run/'mission.mp4',episode/'video/mission.mp4')
        evaluation=dict(success=row['strict_saved_map_success'],reason=row['reason'],
            elapsed_seconds=row['elapsed_sim_s'],targets=len(row['registered_ids']),collisions=row['bumper_events'],
            ground_truth_audit=dict(closest_approach_m=metrics['min_ground_truth_m']))
        (episode/'evaluation.json').write_text(json.dumps(evaluation,indent=2)+'\n')
        (episode/'video/recording.json').write_text(json.dumps(dict(frames=audit['video']['frames'],duration_seconds=audit['video']['duration_s']))+'\n')
        # Plot inputs stay in the local artifact export, not the Git source tree.
        plot=destination/'plot_inputs'/run.name;plot.mkdir(parents=True)
        for file in ('headings.csv','scene.json'):shutil.copy2(run/file,plot/file)
        shutil.copy2(run/'run/simulation/demo_layout.json',plot/'demo_layout.json')
        provenance.append(dict(run=run.name,requested_seed=row['seed'],rendered_seed=scene['seed'],world_sha256=row['world_sha256'],video_sha256=audit['video']['sha256'],
            physical_images=False,source='native Gazebo demo track',camera=[480,480,5],viewing_fps=5))
    (destination/'video_provenance.json').write_text(json.dumps(dict(domain=174,physical_robot_used=False,videos=provenance),indent=2)+'\n')
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    with tarfile.open(destination.with_suffix('.tgz'),'w:gz') as tar:tar.add(destination,arcname=destination.name)
    print(json.dumps(dict(archive=str(destination.with_suffix('.tgz')),videos=5)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('destination',type=Path)
    args=p.parse_args();export(args.root,args.destination)
