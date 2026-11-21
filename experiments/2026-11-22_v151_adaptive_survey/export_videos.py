"""Stage the nine finalized adaptive-survey RGB clips and publish their verified index."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


CAMPAIGN = 'v151-adaptive-survey-1122'
REGRESSION = {112601, 112602, 112603, 112604, 112605}
HELDOUT = {112801, 112802}
ROLES = {'baseline': HELDOUT, 'candidate': REGRESSION | HELDOUT}
REPO = 'peter010905/simul'
TAG = 'simulation-videos-2026-11'


def require(test, message):
    if not test:
        raise ValueError(message)


def load(path):
    return json.loads(path.read_text())


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def digest(path):
    with path.open('rb') as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def read_video(path, info):
    import cv2
    capture = cv2.VideoCapture(str(path))
    try:
        frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        rate = capture.get(cv2.CAP_PROP_FPS)
        require(frames == info['frames'] and frames > 0, 'Frame count ' + str(path))
        require(math.isclose(rate, 5., abs_tol=.01), 'Video FPS ' + str(path))
        require(abs(frames / rate - info['duration_s']) <= .01, 'Video duration ' + str(path))
        for index in (0, frames - 1):
            capture.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = capture.read()
            require(ok and frame.shape[:2] == (480, 480), 'Undecodable or wrong dimensions ' + str(path))
    finally:
        capture.release()


def trajectory_near_ids(run, metric, scene):
    registered = {int(i): float(t) for i, t in metric['registered_s'].items()}
    targets = {int(target['id']): (target['x'], target['y']) for target in scene['targets']}
    near = set()
    with (run / 'trajectory.csv').open(newline='') as stream:
        for sample in csv.DictReader(stream):
            elapsed = float(sample['elapsed_sim_s'])
            if not 0 <= elapsed <= 600:
                continue
            point = float(sample['world_x']), float(sample['world_y'])
            for ident in map(int, sample['registered_ids'].split()):
                if (ident in targets and ident in registered and elapsed >= registered[ident]
                        and math.dist(point, targets[ident]) <= 1.5):
                    near.add(ident)
    return near


def collect(root, recordings, selected=None):
    protocol = load(root / 'protocol.json')
    require(protocol['physical_drive'] is False and protocol['camera'] == [480, 480, 5],
            'Only the documented synthetic 480x480 5 FPS campaign is eligible')
    require(set(protocol['regression']) == REGRESSION and set(protocol['heldout']) == HELDOUT,
            'Unexpected protocol seeds')
    results = []
    for role, expected in ROLES.items():
        if selected and selected[0] != role:
            continue
        phase = root / role
        report = load(phase / 'report.json')
        saved_paths = [Path(p) for p in load(phase / 'runs.json')]
        rows = {int(row['seed']): row for row in report['runs']}
        require(len(rows) == len(report['runs']) and len(saved_paths) == len(set(saved_paths)),
                'Duplicate seed or episode ' + role)
        if selected is None:
            complete = load(phase / 'complete.json')
            require(complete == {'complete': True, 'count': len(expected)} and report['complete'] is True
                    and set(rows) == expected and len(saved_paths) == len(expected), 'Incomplete phase ' + role)
        seeds = [selected[1]] if selected else sorted(expected)
        for seed in seeds:
            require(seed in expected and seed in rows, f'No completed {role} {seed}')
            row = rows[seed]
            split = 'heldout' if seed in HELDOUT else 'track'
            name = f"v151-survey-{role}-{split}-m{seed}-{root.name.split('-')[-1]}"
            require(row['run'] == name, 'Unexpected recorded episode name')
            run = recordings / name
            require(run in saved_paths and run.resolve() == run, 'Episode escaped the specified recordings root')
            require(not (run / 'interruption.json').exists(), 'Interrupted episode ' + name)
            for filename in ('mission.mp4', 'metrics.json', 'audit.json'):
                file = run / filename
                require(file.is_file() and file.resolve().parent == run, 'Missing or indirect input ' + str(file))
            metric, audit = load(run / 'metrics.json'), load(run / 'audit.json')
            world = protocol['worlds'][str(seed)]
            require(re.fullmatch('[0-9a-f]{64}', world)
                    and world == row['world_sha256'] == metric['world_sha256'] == audit['world_sha256'],
                    'World checksum mismatch ' + name)
            scene = load(run / 'scene.json')
            expected_ids = {int(t['id']) for t in scene['targets']}
            enrolled_scene = root / 'fixtures' / f'map-{seed}' / 'scene.json'
            require(enrolled_scene.is_file() and digest(enrolled_scene) == digest(run / 'scene.json')
                    and isinstance(scene['seed'], int) and len(expected_ids) == 3 and audit['run'] == name,
                    'Wrong scenario or audit ' + name)
            ids = sorted(metric['final_ids'])
            near_ids = sorted(trajectory_near_ids(run, metric, scene))
            require(sorted(row['registered_ids']) == ids and metric['reason'] == row['reason']
                    and row['bumper_events'] == metric['bumper_events']
                    and metric['elapsed_sim_s'] == row['elapsed_sim_s'], 'Metrics mismatch ' + name)
            errors = row['map_errors_saved_m']
            saved_map_ok = set(map(int, errors)) == expected_ids and all(0 <= n <= .25 for n in errors.values())
            strict = bool(metric['strict_success'] and saved_map_ok)
            require(strict == row['strict_saved_map_success'], 'Different saved-map success ' + name)
            if strict:
                require(expected_ids == set(ids) == set(near_ids) and metric['policy_sensor_proof_complete']
                        and metric['bumper_events'] == 0, 'Inconsistent strict pass ' + name)
            info = audit['video']
            video = run / 'mission.mp4'
            require(info == row['video'] and info['sha256'] == digest(video), 'Video checksum mismatch ' + name)
            read_video(video, info)
            relative = Path('training_robust/native90') / CAMPAIGN / name / 'video/mission.mp4'
            rec = dict(frames=info['frames'], duration_seconds=info['duration_s'])
            evaluation = dict(success=strict, elapsed_seconds=row['elapsed_sim_s'], targets=len(ids),
                              collisions=row['bumper_events'], reason=row['reason'],
                              ground_truth_audit={'closest_approach_m': metric['min_ground_truth_m'],
                                  'expected_ids': sorted(expected_ids),
                                  'verified_after_registration_ids': sorted(set(near_ids) & set(row['sensor_verified_ids']))},
                              registered_ids=ids, post_registration_near_ids=near_ids,
                              sensor_verified_ids=row['sensor_verified_ids'], saved_errors_m=errors)
            entry = dict(seed=seed, rendered_seed=scene['seed'], scene_sha256=digest(enrolled_scene),
                         role=role, episode=name, world_sha256=world, sha256=info['sha256'],
                         bytes=video.stat().st_size, local_path=str(relative), asset=CAMPAIGN+'__'+name+'.mp4',
                         recording=rec, evaluation=evaluation)
            results.append((video, entry))
    return results


def validate_bundle(bundle):
    files = load(bundle / 'manifest.json')
    roles = [(v['role'], v['seed']) for v in files]
    require(len(roles) == 9 and set(roles) == {(r, s) for r, seeds in ROLES.items() for s in seeds},
            'Bundle does not contain the exact two baseline and seven candidate runs')
    paths = set()
    for v in files:
        name = v['episode']
        split = 'heldout' if v['seed'] in HELDOUT else 'track'
        prefix = f"v151-survey-{v['role']}-{split}-m{v['seed']}-"
        require(re.fullmatch(re.escape(prefix) + '[0-9]+', name), 'Invalid episode name')
        rel = Path('training_robust/native90') / CAMPAIGN / name / 'video/mission.mp4'
        path = bundle / rel
        require(str(rel) == v['local_path'] and v['asset'] == CAMPAIGN+'__'+name+'.mp4'
                and path == path.resolve() and path.is_file() and path.stat().st_size == v['bytes']
                and digest(path) == v['sha256'], 'Invalid or changed staged clip ' + name)
        rec = load(path.parent/'recording.json')
        require(rec == v['recording'] and load(path.parents[1]/'evaluation.json') == v['evaluation'],
                'Staged metadata mismatch ' + name)
        paths.add(path)
    # The general publisher enumerates every native90 video. Reject extras.
    require(paths == set((bundle/'training_robust/native90').rglob('video/mission.mp4')), 'Unexpected videos')
    return files


def export(args):
    root = args.root.resolve()
    recordings = (args.recordings or root.parent/'corridor-board-73bbb05/recordings').resolve()
    selected = None
    if args.command == 'check':
        role, seed = args.episode.split(':')
        require(role in ROLES, 'Unknown role')
        selected = role, int(seed)
    material = collect(root, recordings, selected)
    if selected:
        print(json.dumps(material[0][1], indent=2))
        return
    dest = args.destination.resolve()
    require(not any((p/'.git').exists() for p in (dest, *dest.parents)),
            'Choose an output directory outside a Git checkout')
    entries = [entry for _, entry in material]
    if dest.exists():
        require(validate_bundle(dest) == entries, 'Destination contains another export')
        print(f'Already verified {len(entries)} video files in {dest}')
        return
    dest.parent.mkdir(exist_ok=True, parents=True)
    with tempfile.TemporaryDirectory(dir=dest.parent, prefix='.stage-survey-') as temp:
        stage = Path(temp)/'bundle'
        for video, entry in material:
            target = stage/entry['local_path']
            target.parent.mkdir(parents=True)
            shutil.copyfile(video, target)
            save(target.parent/'recording.json', entry['recording'])
            save(target.parents[1]/'evaluation.json', entry['evaluation'])
        save(stage/'manifest.json', entries)
        validate_bundle(stage)
        stage.rename(dest)
    print(f'Verified {len(entries)} video files in {dest}')


def publish(args):
    bundle = args.bundle.resolve()
    entries = validate_bundle(bundle)
    output = args.index_dir.resolve()
    root = Path(__file__).resolve().parents[2]
    require(output.name != 'repository_snapshot', 'Choose the adaptive experiment directory for the subset index')
    if (output/'videos.json').exists():
        existing = load(output/'videos.json')
        require({v['asset'] for v in existing['videos']} == {v['asset'] for v in entries},
                'The output directory contains an unrelated video index')
    with tempfile.TemporaryDirectory(prefix='publish-survey-', dir=bundle.parent) as temp:
        stage = Path(temp)
        (stage/'scripts').mkdir()
        (stage/'repository_snapshot').mkdir()
        (stage/'README.md').write_text('# Synthetic RGB videos\n')
        script = stage/'scripts/publish_simulation_videos.py'
        shutil.copyfile(root/'scripts'/script.name, script)
        cmd = [sys.executable, str(script), '--repo', REPO, '--tag', TAG, '--data-root', str(bundle)]
        if args.upload:
            cmd.append('--upload')
        subprocess.run(cmd, check=True)
        report = load(stage/'repository_snapshot/videos.json')
    targets = {v['asset']:v for v in entries}
    require(not report['skipped'] and len(report['videos']) == len(targets) and
            set(targets) == {v['asset'] for v in report['videos']}, 'Wrong publisher selection')
    for row in report['videos']:
        v = targets[row['asset']]
        require(row['sha256'] == v['sha256'] and row['bytes'] == v['bytes']
                and row['recording_frames'] == v['recording']['frames']
                and row['original_evaluator_success'] == v['evaluation']['success'], 'Publisher mismatch')
        row.update(role=v['role'], seed=v['seed'], world_sha256=v['world_sha256'],
                   registered_ids=v['evaluation']['registered_ids'],
                   post_registration_near_ids=v['evaluation']['post_registration_near_ids'])
    if report['uploaded_verified'] != len(targets):
        require(not args.upload, 'Missing remote checksum')
        print(f"Eligible 9, already published and verified {report['uploaded_verified']}.")
        return
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output) as tmp:
        result = Path(tmp)/'videos.json'
        save(result, report)
        result.replace(output/'videos.json')
    print(f"Verified nine remote clips. Subset index: {output/'videos.json'}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(required=True, dest='command')
    for action in ('check', 'export'):
        sub = subs.add_parser(action)
        sub.add_argument('root', type=Path)
        if action == 'check':
            sub.add_argument('episode', help='baseline:112801 or candidate:112601')
        else:
            sub.add_argument('destination', type=Path)
        sub.add_argument('--recordings', type=Path)
    sub = subs.add_parser('publish')
    sub.add_argument('bundle', type=Path)
    sub.add_argument('index_dir', type=Path, help='Receives only verified JSON; use the experiment directory')
    sub.add_argument('--upload', action='store_true')
    args = parser.parse_args()
    if args.command == 'publish':
        publish(args)
    else:
        export(args)
