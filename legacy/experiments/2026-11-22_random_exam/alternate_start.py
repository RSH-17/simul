"""Enroll a second starting pose in the middle official world before either policy drives there."""
import argparse
import hashlib
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

import numpy as np
from scipy import ndimage

from fixture import FILES, digest, read, save
from geometry import RESOLUTION, raster, table_parts


def choose_start(scene, tables, seed):
    parts = table_parts(tables, scene['furniture'])
    body, origin = raster(scene, parts, for_spawn=True)
    wheel, _ = raster(scene, parts)

    def distance(grid):
        return ndimage.distance_transform_edt(np.pad(~grid, 1))[1:-1, 1:-1] * RESOLUTION

    room_clearance = distance(body)
    labels, _ = ndimage.label(distance(wheel) >= .40)
    original = scene['robot_spawn']
    rc = np.floor((np.array([original['x'], original['y']]) - origin) / RESOLUTION).astype(int)[::-1]
    region = labels[tuple(rc)]
    if not region:
        raise ValueError('The original start has no connected driving region.')
    cells = np.argwhere((labels == region) & (room_clearance >= .40) & (room_clearance < .80))
    if not len(cells):
        raise ValueError('No start in the registered clearance band.')
    rng = np.random.default_rng(seed)
    row, col = cells[rng.integers(len(cells))]
    x, y = origin + (np.array([col, row]) + .5) * RESOLUTION
    pose = dict(x=float(x), y=float(y), yaw=float(rng.uniform(-math.pi, math.pi)))
    return pose, float(room_clearance[row, col]), len(cells)


def prepare(parent, output):
    parent, output = parent.resolve(), output.resolve()
    if output.exists():
        raise FileExistsError(f'Keep the existing enrollment in {output}')
    protocol = read(parent/'protocol.json')
    registration = read(parent/'registration.json')
    if (registration['known_marker_count'] != 3 or registration['furniture'] != 'fixed'
            or len(protocol['worlds']) != 5 or digest(parent/'registration.json') != protocol['registration_sha256']):
        raise ValueError('Expected the enrolled fixed-desk, three-marker, five-layout panel.')
    world = protocol['worlds'][len(protocol['worlds']) // 2]
    original = parent/'fixtures'/f"map-{world['seed']}"
    for name, expected in world['files'].items():
        if digest(original/name) != expected:
            raise ValueError('Changed source fixture: '+name)
    domain = '8780-fixed-desk-secondary-start-v1'
    protocol_hash = digest(parent/'protocol.json')
    nonce = hashlib.sha256(f'{protocol_hash}/{domain}'.encode()).hexdigest()
    seed = int(nonce[:16], 16)
    scene = read(original/'scene.json')
    pose, clearance, choices = choose_start(scene, read(original/FILES[3])['tables'], seed)
    start = [pose[k] for k in ('x', 'y', 'yaw')]
    metadata = dict(parent_protocol_sha256=protocol_hash, geometry_seed=world['seed'],
                    selection='middle enrolled map; uniform cell among connected starts with 0.40 <= clearance < 0.80 m; uniform yaw',
                    pose_seed=seed, qualifying_cells=choices, grid_clearance_m=clearance,
                    original_start=world['start'], new_start=start, generator_sha256=digest(Path(__file__)))
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as temp:
        root = Path(temp)/'enrolled'
        fixture = root/'fixtures'/original.name
        shutil.copytree(original, fixture)
        scene.update(robot_spawn=pose, robot_start_expected=pose)
        save(fixture/'scene.json', scene)
        shutil.copyfile(fixture/'scene.json', fixture/'run/simulation/scene.json')
        launch = read(fixture/'scene.yaml')
        launch['start'] = start
        save(fixture/'scene.yaml', launch)
        files = {name: digest(fixture/name) for name in FILES}
        for name in ('world/osp_studio.sdf', 'run/simulation/demo_layout.json', 'run/simulation/demo_floor.png'):
            if files[name] != world['files'][name]:
                raise ValueError('Changed room, obstacles, prints or furniture: '+name)
        sampled = dict(world, start=start, files=files)
        save(root/'registration.json', dict(registration, nonce=nonce, draws=[world['seed']],
                                          repeated_index=None, alternate_start=metadata))
        save(root/'protocol.json', dict(registration_sha256=digest(root/'registration.json'),
                                        worlds=[sampled], plan=[dict(pair=1, world=0, repeat=False,
                                                                     order=['candidate', 'baseline'])]))
        save(root/'start-comparison.json', metadata)
        root.rename(output)
    return metadata


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('official_panel', type=Path)
    p.add_argument('separate_output', type=Path)
    args = p.parse_args()
    print(prepare(args.official_panel, args.separate_output))
