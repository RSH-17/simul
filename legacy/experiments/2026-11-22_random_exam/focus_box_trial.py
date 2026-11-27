"""Run only the new focus policy on the exact worlds of the five-box comparison."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import time

from visual_survey_trial import native, read, save, sha


TITLE = 'Mixed v1.5.2.1 Focus Continuity'


def enroll(panel, reference, workspace, freeze_file, engine):
    reference_protocol = read(reference / 'protocol.json')
    original = read(reference / 'registration.json')
    freeze = read(freeze_file)
    worlds = reference_protocol['worlds']
    protocol_digest = sha(reference / 'protocol.json')
    if (protocol_digest != freeze['protocol_sha256'] or
            reference_protocol['registration_sha256'] != sha(reference / 'registration.json') or
            original['known_marker_count'] != 3 or original['furniture'] != 'fixed' or
            original['camera'] != [480, 480, 5] or original['duration_s'] != 600 or
            original['limits']['obstacle_geometry'] != 'original_exam_boxes' or
            original['survey_distant_hints'] or len(worlds) != 5 or
            len({w['seed'] for w in worlds}) != 5):
        raise ValueError('The registered five-box exam and its camera settings are required')
    manifest = read(workspace / 'focus_continuity_manifest.json')
    if (freeze['patch_sha256'] != manifest['patch_sha256'] or
            freeze['parent_mission_sha256'] != manifest['parent_mission_sha256'] or
            manifest['mission_sha256'] != sha(workspace / 'src/osp_mission/osp_mission/mission.py')):
        raise ValueError('The policy changed after its freeze')
    candidate = engine.identity(workspace)
    if (candidate['source_manifest_sha256'] == original['candidate']['source_manifest_sha256'] or
            engine.file_identity() != original['checksums']):
        raise ValueError('Expected the new policy on the original native engine')
    seen = {r['seed'] for r in freeze['prior_results']}
    seen.add(freeze['known_first_run']['seed'])
    if not seen <= {w['seed'] for w in worlds}:
        raise ValueError('Freeze record refers to other worlds')
    cases = []
    for index, world in enumerate(worlds, 1):
        if world['movable_objects'] != 5 or list(world['object_shapes'].values()).count('box') != 5:
            raise ValueError('A world contains different obstacles')
        engine.paired.checked_files(reference / 'fixtures' / f"map-{world['seed']}", world)
        cases.append(dict(pair=index, world=index-1, repeat=0, order=['candidate']))
    reg = dict(original, candidate=candidate, candidate_label=TITLE,
               nonce=hashlib.sha256((protocol_digest + candidate['source_manifest_sha256']).encode()).hexdigest(),
               reference=str(reference), reference_protocol_sha256=protocol_digest,
               diagnostic_seeds=sorted(seen), decision_record=freeze,
               fresh_role='candidate', reused_role='baseline', repeated_index=None)
    if panel.exists():
        raise FileExistsError(panel)
    panel.mkdir(parents=True)
    save(panel / 'registration.json', reg)
    (panel / 'fixtures').symlink_to(reference / 'fixtures', target_is_directory=True)
    save(panel / 'protocol.json', dict(registration_sha256=sha(panel / 'registration.json'),
                                     worlds=worlds, plan=cases))
    save(panel / 'status.json', dict(state='waiting_for_reference', new_drives=0))


def controls(panel, reference, engine):
    reg = read(panel / 'registration.json')
    protocol = read(panel / 'protocol.json')
    reference_scores = read(reference / 'comparison.json')
    if (reg['reference_protocol_sha256'] != sha(reference / 'protocol.json') or
            reference_scores['valid_pairs'] != len(read(reference / 'protocol.json')['plan'])):
        raise ValueError('The original matched comparison is incomplete or changed')
    chosen = []
    recordings = engine.runner.HOME / 'corridor-board-73bbb05/recordings'
    for world, pair in zip(protocol['worlds'],
                           [p for p in reference_scores['pairs'] if not p['repeat']], strict=True):
        if not pair['valid'] or pair['seed'] != world['seed']:
            raise ValueError('Invalid reference world')
        for role in ('baseline', 'candidate'):
            row = pair[role]
            if (not row['score']['valid'] or row['world_sha256'] != world['files']['world/osp_studio.sdf'] or
                    any(sha(recordings / row['name'] / key) != expected for key, expected in row['evidence'].items())):
                raise ValueError('The original reference evidence changed')
        chosen.append(pair['baseline'])
    return chosen


def run(panel, reference, engine):
    lock = (panel / 'owner.lock').open('a')
    with lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            state = read(reference / 'status.json')['state']
            if state == 'complete':
                break
            if state in ('invalid_pair', 'infrastructure_hold'):
                raise RuntimeError('Reference stopped: ' + state)
            time.sleep(25)
        baseline = controls(panel, reference, engine)
        ledger = panel / 'episodes.json'
        if ledger.exists():
            if [r for r in read(ledger) if r['role'] == 'baseline'] != baseline:
                raise ValueError('The reused controls differ from the signed reference')
        else:
            save(ledger, baseline)
        engine.run_all(panel)
        if read(panel / 'status.json')['state'] != 'complete':
            raise ValueError('A new drive failed the measurement acceptance criteria')
        expected = {r['pair']: r for r in read(reference / 'episodes.json') if r['role'] == 'candidate'}
        from visual_survey_results import matched, endpoint
        from board_results import apply
        for row in read(ledger):
            if row['role'] == 'candidate':
                matched(endpoint(panel, row), endpoint(reference, expected[row['pair']]))
                apply(Path(os.environ['CAMERA_BENCH_CARDS']) / row['name'], row, TITLE)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('enroll', 'run'))
    for arg in ('panel', 'reference', 'workspace', 'freeze', 'native_exam'):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    engine = native(args.native_exam.resolve())
    if args.action == 'enroll':
        enroll(args.panel, args.reference, args.workspace, args.freeze, engine)
    else:
        try:
            run(args.panel, args.reference, engine)
        except Exception as error:
            save(args.panel / 'status.json', dict(state='infrastructure_hold', error=repr(error)))
            raise
