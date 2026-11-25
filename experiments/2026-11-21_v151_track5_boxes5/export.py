"""Export completed five-track results and synthetic videos, never raw ROS bags."""
import argparse
import collections
import hashlib
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
    report['boxes_per_layout']=5
    destination.mkdir(exist_ok=False)
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for name in ('protocol.json','runtime_parameters.json'):shutil.copy2(root/name,destination/name)
    if (root/'supervisor_revision.json').exists():shutil.copy2(root/'supervisor_revision.json',destination/'supervisor_revision.json')
    provenance=[]
    for name,row in zip(json.loads((root/'runs.json').read_text()),report['runs']):
        run=Path(name);assert run.name==row['run']
        metrics=json.loads((run/'metrics.json').read_text());audit=json.loads((run/'audit.json').read_text())
        scene=json.loads((run/'scene.json').read_text())
        assert len(scene['obstacles'])==5 and scene['room_size']==[20.15,11.35]
        row['rendered_seed']=scene['seed']
        row['wall_marker_ids']=sorted(t['id'] for t in scene['targets'] if t['attached_to'].startswith('wall_'))
        row['box_marker_ids']=sorted(t['id'] for t in scene['targets'] if not t['attached_to'].startswith('wall_'))
        row['marker_sizes_m']={str(t['id']):t['size'] for t in scene['targets']}
        row['independent_closest_m']=metrics['min_ground_truth_m']
        row['registered_s']=metrics['registered_s']
        row['registered_and_within_s']=metrics['registered_and_within_s']
        geometry={k:scene[k] for k in ('room_size','obstacles','targets','robot_spawn')}
        row['geometry_sha256']=hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest()
        scheduler=run/'native/center_wall.jsonl'
        if scheduler.exists():
            events=[json.loads(line) for line in scheduler.read_text().splitlines()]
            row['scheduler_phases']=dict(collections.Counter(e['phase'] for e in events))
            row['scheduler_last_available']=events[-1]['available'] if events else {}
            row['scheduler_reasons']=dict(collections.Counter(e['reason'] for e in events))
            row['weak_wall_goal_selections']=sum(bool(e.get('weak_wall')) for e in events)
            wall_events=[e for e in events if e['kind']=='wall_surface']
            row['first_wall_surface_selection_s']=wall_events[0]['elapsed'] if wall_events else None
            # Dispatched selection records, not independent Nav2 path feasibility.
            row['late_interior_before_wall_budget']=[dict(elapsed=e['elapsed'],available=e['available'])
                for e in events if e['reason']=='unseen_reachable_object'
                and e['available'].get('wall',0)>0
                and 600-e['elapsed']<=min(.15*600,12+5*e['available']['wall'])]
        episode=destination/'training_robust/native90/v151-track-five-boxes5-1121'/run.name
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
    assert len({r['geometry_sha256'] for r in report['runs']})==5
    for file in [root/'camera_geometry.json',root/'wall-glimpse-replay.json',root/'scheduler_integrity.json',*root.glob('wall-audit-*.json')]:
        if file.exists():shutil.copy2(file,destination/file.name)
    (destination/'video_provenance.json').write_text(json.dumps(dict(domain=174,physical_robot_used=False,videos=provenance),indent=2)+'\n')
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    with tarfile.open(destination.with_suffix('.tgz'),'w:gz') as tar:tar.add(destination,arcname=destination.name)
    print(json.dumps(dict(archive=str(destination.with_suffix('.tgz')),videos=5)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('destination',type=Path)
    args=p.parse_args();export(args.root,args.destination)
