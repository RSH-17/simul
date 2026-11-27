"""Enroll matched trials in the actual 8780 demo room, including the pictured layout."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import sys
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import yaml


HOME = Path.home()
HERE = Path(__file__).resolve().parent
SURVEY = HERE.parent / '2026-11-22_v151_adaptive_survey'
ANCHOR = HOME / 'track-recordings/.layouts/uFjtqfyJ'
ANCHOR_SEED = 2983619014
ARCHIVE = HOME / 'v151-track-five-boxes5-1795247134501974510'
STATIC_SHA = 'beff18e9879eaa96cc036e4967575987282c1d951f6d9488b145e4d59f080c6f'
LAYOUT_SHA = '55146394e454573274347edf6f38d01d4c52ff14fd12d009233855e176d6b557'
FLOOR_SHA = '3fd7531e7558cba98a9a8ba1075ecb45310212745a506591c0d8aeda8ec9a228'
ANCHOR_SHA = '589d152dcfef420f6fda37fccdc30c3d73e9f90ebbdab8359d04bf94a5538b1a'
FILES = ('world/osp_studio.sdf', 'scene.json', 'scene.yaml', 'run/simulation/demo_layout.json',
         'run/simulation/demo_floor.png')
WORKSPACES = dict(baseline='flow_v151_exposure_sync_r2_ws', R3='v151_object_followthrough1122_r3_ws',
                  R4='v151_tiny_named_relook1122_r4_ws')


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read(path):
    return json.loads(path.read_text())


def save(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temp.replace(path)


def inspect_world(folder, label, count):
    scene = read(folder / 'scene.json')
    params = yaml.safe_load((folder / 'scene.yaml').read_text())
    if (scene['appearance'] != 'demo_room' or scene['room_size'] != [20.15, 11.35] or
            len(scene['obstacles']) != count or params['duration'] != 600):
        raise ValueError(f'Unexpected room, object count or duration in {folder}')
    if params['targets'] != scene['targets'] or list(params['start']) != [scene['robot_spawn'][k] for k in ('x','y','yaw')]:
        raise ValueError('Launch inputs disagree with fixture metadata')
    boxes = {b['name'] for b in scene['obstacles']}
    targets = scene['targets']
    hosts = {t['attached_to'] for t in targets}
    if (len(targets) != 3 or {t['id'] for t in targets} != {7,32,103} or
            len(hosts & boxes) != 2 or sum(s.startswith('wall_') for s in hosts) != 1):
        raise ValueError('Expected two marked boxes and a marked wall')
    tree = ET.parse(folder / FILES[0]).getroot()
    world = tree.find('world')
    moving = boxes | {f"target_{t['id']}" for t in targets}
    for model in list(world.findall('model')):
        if model.get('name') in moving:
            moving.remove(model.get('name'))
            world.remove(model)
    if moving:
        raise ValueError(f'Missing actual Gazebo objects {moving}')
    textures = list(tree.iter('albedo_map'))
    if len(textures) != 1:
        raise ValueError('Unexpected floor texture count')
    asset = urlparse(textures[0].text)
    floor = folder / FILES[-1]
    if asset.scheme != 'file' or asset.netloc or not Path(unquote(asset.path)).samefile(floor):
        raise ValueError('The renderer is loading a different floor')
    textures[0].text = 'demo_floor.png'
    unchanged_xml = ET.canonicalize(ET.tostring(tree, encoding='unicode'), strip_text=True)
    static_digest = hashlib.sha256(unchanged_xml.encode()).hexdigest()
    files = {name: sha(folder / name) for name in FILES}
    if (static_digest, files[FILES[-2]], files[FILES[-1]]) != (STATIC_SHA, LAYOUT_SHA, FLOOR_SHA):
        raise ValueError('Static world, furniture, physics or floor differs from the screenshot world')
    if label == 'pictured':
        anchors = {7: (-8.775,-2.355), 32: (-7.225,5.600), 103: (6.300,.710)}
        if (files[FILES[0]] != ANCHOR_SHA or scene['seed'] != ANCHOR_SEED or
                any(math.dist((t['x'],t['y']), anchors[t['id']]) > .002 for t in targets)):
            raise ValueError('The pictured layout does not match the original recording')
    return dict(seed=scene['seed'], label=label, boxes=count, start=params['start'],
                files=files, fixed_room_sha256=static_digest)


def pair_plan(worlds):
    # Anchor twice up front so its paired evidence stays reusable if the shortlist fails.
    # Two distinct orders also expose timing variation on this previously seen layout.
    indices = (0,0,1,2,3,4,0,1,3)
    seen = {}
    plan = []
    for number, index in enumerate(indices, 1):
        seed = worlds[index]['seed']
        repeat = seen.get(seed, 0)
        candidate_first = (index % 2 + repeat) % 2
        order = ['candidate','baseline'] if candidate_first else ['baseline','candidate']
        plan.append(dict(pair=number, seed=seed, repeat=repeat, order=order))
        seen[seed] = repeat + 1
    return plan


def prepare(root):
    sys.path.insert(0, str(SURVEY / 'validation'))
    from validate import identity
    root.mkdir(parents=True, exist_ok=True)
    fixtures = root / 'fixtures'
    fixtures.mkdir(exist_ok=True)
    original = inspect_world(ANCHOR, 'pictured', 4)
    source = {k: identity(HOME / ws) for k, ws in WORKSPACES.items()}
    builder = HOME / 'track-resume-5e95333/2026-11-19_random_track/prepare.py'
    reference = HOME / 'track-reference-73bbb05'
    template = ARCHIVE / 'reference5'
    geometry_inputs = {str(p): sha(p) for p in (builder, template/'resources/demo_boxes_walls.yaml',
                       reference/'osp_project/sim_world.py', reference/'osp_project/demo_room.py')}
    execution = {p.name: sha(p) for p in (HERE/'prepare.py', HERE/'paired.py', SURVEY/'run.py',
                 SURVEY/'trial.py', SURVEY/'validation/validate.py')}
    selection_files = {p.name: sha(p) for p in (HERE/'select_finalist.py', HERE/'queue.py',
                       HERE.parent/'2026-11-22_v151_tiny_named_relook/track_screen.py')}
    enrollment = root / 'enrollment.json'
    if enrollment.exists():
        record = read(enrollment)
        if (record['policies'] != source or record['generator'] != geometry_inputs or
                record['execution'] != execution or record['selection_files'] != selection_files):
            raise ValueError('Registered inputs or policy build changed')
    else:
        nonce = secrets.token_hex(24)
        draws = [int(hashlib.sha256(f'{nonce}-{i}'.encode()).hexdigest()[:8], 16) for i in range(4)]
        record = dict(nonce=nonce, requested_draws=draws, policies=source, generator=geometry_inputs,
                      execution=execution, selection_files=selection_files)
        save(enrollment, record)
    planfile = root / 'protocol.json'
    if planfile.exists():
        old = read(planfile)
        verified = [inspect_world(fixtures / f"map-{w['seed']}", w['label'], w['boxes']) for w in old['worlds']]
        if verified != old['worlds'] or old['plan'] != pair_plan(verified) or old['policies'] != source:
            raise ValueError('Frozen exam fixtures or order changed')
        return old
    known = {ANCHOR_SEED, *range(112601, 112606)}
    for scene_file in (HOME/'track-recordings/.layouts').glob('*/scene.json'):
        known.add(read(scene_file)['seed'])
    prior = [p.name for p in (HOME/'corridor-board-73bbb05/recordings').iterdir()]
    anchor_link = fixtures / f'map-{ANCHOR_SEED}'
    if not anchor_link.exists():
        anchor_link.symlink_to(ANCHOR, target_is_directory=True)
    worlds = [original]
    for index, draw in enumerate(record['requested_draws'], 1):
        folder = fixtures / f'draw-{index}'
        if not folder.exists():
            folder.mkdir()
            env = dict(os.environ, PYTHONPATH=str(reference) + os.pathsep + os.environ.get('PYTHONPATH',''))
            subprocess.run([sys.executable, str(builder), str(template), str(folder), '--seed', str(draw)],
                           env=env, check=True, stdout=subprocess.DEVNULL)
        world = inspect_world(folder, f'unused_exam_layout_{index}', 5)
        seed = world['seed']
        if seed in known or any(f'-m{seed}-' in name for name in prior):
            raise ValueError('Repeated evaluation seed; retain enrollment and report the conflict')
        known.add(seed)
        target = fixtures / f'map-{seed}'
        if not target.exists():
            target.symlink_to(folder, target_is_directory=True)
        if target.resolve() != folder.resolve():
            raise ValueError('Different fixture at an enrolled seed')
        worlds.append(world)
    protocol = dict(**record, worlds=worlds, plan=pair_plan(worlds), anchor_seed=ANCHOR_SEED,
                    physical_drive=False, domain=174, camera=[480,480,5], duration_s=600,
                    selection='Only five original large development tracks select a finalist. Auxiliary rooms have zero vote.',
                    independence='The pictured layout was provided by the user. Four draws after policy freeze use the same upstream room generator; track repeats are not additional independent layouts.')
    save(planfile, protocol)
    return protocol


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--inspect-original', action='store_true')
    args = parser.parse_args()
    if args.inspect_original:
        print(json.dumps(inspect_world(ANCHOR, 'pictured', 4), indent=2))
    else:
        args.root.mkdir(parents=True, exist_ok=True)
        with (args.root / 'prepare.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = prepare(args.root.resolve())
        print('Frozen exam panel:', len(data['worlds']), 'layouts,', len(data['plan']), 'pairs.',
              'Protocol SHA-256', sha(args.root / 'protocol.json'))
