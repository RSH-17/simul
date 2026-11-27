"""Export small reports and finalized videos; keep raw frames and ROS logs in VM."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def export(root, output):
    output.mkdir(parents=True, exist_ok=False)
    report = output / 'reports'
    report.mkdir()
    for name in ('comparison.json', 'screening.json', 'replay-map-1.json',
                 'replay-map-2.json', 'real_camera_reference.json',
                 'suite_complete.json', 'followup_complete.json', 'followup_plan.json'):
        shutil.copy2(root / name, report / name)
    stage = Path.home() / 'flow_v12_camera_bench_ws'
    original = Path.home() / 'flow_v3_r32_marker_relook_ws'
    hashes = json.loads((stage / 'frozen_v12_sha256.json').read_text())
    for relative, expected in hashes.items():
        for workspace in (stage, original):
            actual = hashlib.sha256((workspace / 'src/osp_mission' / relative).read_bytes()).hexdigest()
            assert actual == expected, (workspace, relative)
    (report / 'source_integrity.json').write_text(json.dumps(dict(
        checked_files=len(hashes), unchanged=True,
        only_excluded_module='osp_mission/sensor_model.py', sha256=hashes), indent=2)+'\n')
    refs = json.loads((root / 'reference_clips.json').read_text())
    runs = json.loads((root / 'runs.json').read_text())
    for name in refs + runs:
        run = Path(name)
        audit = json.loads((run / 'audit.json').read_text())
        episode = output / 'training_robust/native90/camera-v12' / run.name
        (episode / 'video').mkdir(parents=True)
        shutil.copy2(run / 'mission.mp4', episode / 'video/mission.mp4')
        shutil.copy2(run / 'audit.json', episode / 'camera_audit.json')
        shutil.copy2(run / 'metrics.json', episode / 'camera_metrics.json')
        approached = audit['registered_and_within_s']
        success = all(str(marker) in approached for marker in (7, 32, 103))
        evaluation = dict(success=success, reason=audit['reason'],
            elapsed_seconds=audit['elapsed_sim_s'], targets=len(audit['final_ids']),
            collisions=audit['bumper_events'],
            protocol='IDs registered and independent distance <=1.5m; not policy sensor-witness completion',
            ground_truth_audit=dict(closest_approach_m=audit['min_ground_truth_m']))
        (episode / 'evaluation.json').write_text(json.dumps(evaluation, indent=2)+'\n')
        (episode / 'video/recording.json').write_text(json.dumps(dict(
            frames=audit['video']['frames'], duration_seconds=audit['video']['duration_s']), indent=2)+'\n')
    archive = output.with_suffix('.tgz')
    with tarfile.open(archive, 'w:gz') as target:
        target.add(output, arcname=output.name)
    print(archive)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    export(args.root, args.output)
