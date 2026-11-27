"""Export finalized matched development evidence and synthetic RGB videos."""
import argparse
import collections
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import tarfile

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('wall_export_summary', HERE.parent/'2026-11-21_camera_aware_policy/summarize.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def wall_looks(run, start):
    path = run/'native/near_pass_inspections.jsonl'
    events = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    looks, active = [], None
    for event in events:
        if event['event'] == 'requested':
            assert active is None
            active = dict(started_clock_s=event['stamp'], elapsed_mission_s=event['stamp']-start,
                choice=event['choice'], finished=False, processed_frames=None, fresh_frame_minimum_met=None)
            looks.append(active)
        elif event['event'] == 'wall_view_frames' and active is not None:
            active.update(processed_frames=event['processed_frames'],
                fresh_frame_minimum_met=event['minimum_met'], dwell_s=event['dwell_s'])
        elif event['event'] == 'finished' and active is not None:
            active.update(finished=True, reason=event['reason'], resume_requested=event['resumed_navigation'],
                finished_clock_s=event['stamp'], actual_observation_s=event['stamp']-active['started_clock_s'])
            active = None
    with (run/'headings.csv').open() as stream:
        poses = list(csv.DictReader(stream))
    for look in looks:
        if not look['finished']:
            continue
        end = look['finished_clock_s']-start
        points = [(float(p['x']), float(p['y'])) for p in poses if end <= float(p['elapsed_sim_s']) <= end+10]
        look['independent_motion_next_10_sim_s_m'] = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    text = (run/'launch.log').read_text(errors='replace')
    return dict(requests=len(looks), completed_fresh_frame_dwells=sum(
            l['finished'] and l.get('reason') == 'processed_frame_dwell' and l['fresh_frame_minimum_met'] is True for l in looks),
        requested_navigation_resumptions=sum(l.get('resume_requested', False) for l in looks),
        actual_observation_s=sum(l.get('actual_observation_s', 0.) for l in looks),
        budget_charged_s=sum(max(l.get('actual_observation_s', 0.), l['choice']['estimated_sim_s']) for l in looks),
        mission_navigation_resume_log_count=text.count('Resuming nav after sensor recovery'),
        event_counts=dict(collections.Counter(e['event'] for e in events)), history=looks,
        limitations='A resume_requested flag is intent; resume log count and subsequent independent motion are separate evidence. '
            'Planned surface gain is not credited camera coverage. Final native result.json is not required by the legacy collector; these counts use the saved runtime event journal.')


def export(root, baseline_export, destination):
    assert (root/'complete.json').exists()
    report = summary.analyze(root)
    protocol = json.loads((root/'protocol.json').read_text())
    assert report['complete'] and len(report['runs']) == len(protocol['seeds'])
    baseline = json.loads((baseline_export/'report.json').read_text())
    destination.mkdir(exist_ok=False)
    report['notes'][0] = 'Development regression on previously analyzed baseline worlds; not a held-out success-rate estimate.'
    report['comparisons'] = []
    for group in report['groups']:
        group['split'] = 'development_regression'
    provenance = []
    for path, row in zip(json.loads((root/'runs.json').read_text()), report['runs']):
        run = Path(path)
        assert run.name == row['run']
        old = next(r for r in baseline['runs'] if r['seed'] == row['seed'])
        assert row['world_sha256'] == old['world_sha256']
        metrics = json.loads((run/'metrics.json').read_text())
        audit = json.loads((run/'audit.json').read_text())
        scene = json.loads((run/'scene.json').read_text())
        assert len(scene['obstacles']) == 5
        row.update(split='development_regression', rendered_seed=scene['seed'],
            wall_marker_ids=old['wall_marker_ids'], box_marker_ids=old['box_marker_ids'],
            independent_closest_m=metrics['min_ground_truth_m'], registered_s=metrics['registered_s'],
            registered_and_within_s=metrics['registered_and_within_s'],
            wall_looks=wall_looks(run, row['mission_start_clock_s']))
        first = next((l for l in row['wall_looks']['history'] if l['fresh_frame_minimum_met'] is True), None)
        if first is not None:
            with (run/'frames.csv').open() as stream:
                frames = list(csv.DictReader(stream))
            desired = first['finished_clock_s']-row['mission_start_clock_s']-.4
            frame = min(frames, key=lambda f: abs(float(f['elapsed_sim_s'])-desired))
            preview = destination/(run.name+'-wall-look.jpg')
            shutil.copy2(run/'camera'/f"{int(frame['frame']):05d}.jpg", preview)
            row['wall_look_preview'] = dict(file=preview.name, frame=int(frame['frame']),
                elapsed_sim_s=float(frame['elapsed_sim_s']), saved_jpeg=True)
        events = [json.loads(line) for line in (run/'native/center_wall.jsonl').read_text().splitlines()]
        row['scheduler_phases'] = dict(collections.Counter(e['phase'] for e in events))
        row['scheduler_reasons'] = dict(collections.Counter(e['reason'] for e in events))
        row['first_wall_surface_selection_s'] = next((e['elapsed'] for e in events if e['kind'] == 'wall_surface'), None)
        row['first_reserved_wall_selection_s'] = next((e['elapsed'] for e in events
            if e['reason'] == 'remaining_time_for_mapped_walls'), None)
        row['late_fresh_interior_wall_budget_bypass'] = [e for e in events if e['reason'] == 'unseen_reachable_object'
            and e['available'].get('wall', 0) > 0 and 600-e['elapsed'] <= min(90, 12+5*e['available']['wall'])]
        report['comparisons'].append(dict(seed=row['seed'], same_world_sha256=True, baseline=old, candidate=row))
        episode = destination/'training_robust/native90/v152-wall-transit-r2-1121'/run.name
        (episode/'video').mkdir(parents=True)
        shutil.copy2(run/'mission.mp4', episode/'video/mission.mp4')
        assert hashlib.sha256((episode/'video/mission.mp4').read_bytes()).hexdigest() == audit['video']['sha256']
        evaluation = dict(success=row['strict_saved_map_success'], reason=row['reason'],
            elapsed_seconds=row['elapsed_sim_s'], targets=len(row['registered_ids']), collisions=row['bumper_events'],
            ground_truth_audit=dict(closest_approach_m=metrics['min_ground_truth_m']))
        (episode/'evaluation.json').write_text(json.dumps(evaluation, indent=2)+'\n')
        (episode/'video/recording.json').write_text(json.dumps(dict(frames=audit['video']['frames'],
            duration_seconds=audit['video']['duration_s']))+'\n')
        plot = destination/'plot_inputs/candidate'/run.name
        plot.mkdir(parents=True)
        for file in ('headings.csv', 'scene.json'):
            shutil.copy2(run/file, plot/file)
        shutil.copy2(run/'run/simulation/demo_layout.json', plot/'demo_layout.json')
        shutil.copytree(baseline_export/'plot_inputs'/old['run'], destination/'plot_inputs/baseline'/old['run'])
        provenance.append(dict(run=run.name, world_sha256=row['world_sha256'],
            video_sha256=audit['video']['sha256'], camera=[480, 480, 5], physical_images=False))
    for name in ('protocol.json', 'runtime_parameters.json'):
        shutil.copy2(root/name, destination/name)
    if (root/'observer_revision.json').exists():
        shutil.copy2(root/'observer_revision.json', destination/'observer_revision.json')
    original = json.loads((baseline_export/'runtime_parameters.json').read_text())
    actual = json.loads((root/'runtime_parameters.json').read_text())
    unchanged = [name for name in original if name.startswith('/')]
    assert all(original[name] == actual[name] for name in unchanged)
    report['runtime_parameters_unchanged_vs_baseline'] = unchanged
    (destination/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (destination/'video_provenance.json').write_text(json.dumps(dict(videos=provenance,
        domain=174, physical_robot_used=False), indent=2)+'\n')
    with tarfile.open(destination.with_suffix('.tgz'), 'w:gz') as tar:
        tar.add(destination, arcname=destination.name)
    print(json.dumps(dict(archive=str(destination.with_suffix('.tgz')), videos=len(provenance))))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('baseline_export', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    export(a.root, a.baseline_export, a.destination)
