"""Evaluator-only two new compact layouts with physical partitions and occlusion."""
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET



def partition(x, y, sx, sy):
    return dict(x=x, y=y, sx=sx, sy=sy, yaw=0., height=1.2)


def box(x, y, sx, sy):
    return dict(type="box", x=x, y=y, sx=sx, sy=sy, height=.85, yaw=0.)


def marker(ident, x, y, angle, host):
    return dict(id=ident, x=x, y=y, size=.16, yaw=angle, attached_to=host)


def scenario(seed):
    pi = math.pi
    if seed == 112801:
        # Full-height internal baffles force a slalom around opposite ends.
        walls = [partition(3., 1.9, .14, 3.8), partition(5.9, 4., .14, 4.)]
        obstacles = [box(1.85, 1.65, .60, .70), box(7.55, 4.15, .70, .70)]
        targets = [marker(32, 1.85, 2.015, pi/2, "box_0"),
                   marker(103, 7.55, 3.785, -pi/2, "box_1"),
                   marker(7, 7.35, .091, pi/2, "wall_south")]
        start = [1.2, 3.0, 0.]
        description = 'offset baffles'
    elif seed == 112802:
        # Three partitions form a side alcove. The wall target faces into it.
        walls = [partition(4.5, 3.95, 3.0, .14), partition(3.05, 2.95, .14, 2.0),
                 partition(5.95, 2.95, .14, 2.0)]
        obstacles = [box(1.85, 4.7, .8, .65), box(7.5, 2.65, .65, .8),
                     dict(type='cylinder', x=6.8, y=4.9, radius=.24, height=.85)]
        targets = [marker(7, 4.4, 3.865, -pi/2, 'wall_alcove'),
                   marker(32, 2.265, 4.7, 0., 'box_0'),
                   marker(103, 7.5, 3.065, pi/2, 'box_1')]
        start = [1.2, 1.25, .2]
        description = 'recessed alcove'
    else:
        raise ValueError(seed)
    outer = [partition(4.5, 0., 9.0, .14), partition(4.5, 6., 9.0, .14),
             partition(0., 3., .14, 6.), partition(9., 3., .14, 6.)]
    return dict(seed=seed, randomize=False, start=start,
                arena=dict(width_m=9., height_m=6., walls=outer+walls,
                           polygon=[[0,0],[9,0],[9,6],[0,6]]),
                obstacles=obstacles, targets=targets), description


def make(parent, seed):
    from osp_mission.world import generate
    import yaml
    cfg, topology = scenario(seed)
    dst = parent / f'map-{seed}'
    dst.mkdir()
    source = dst / 'world_source.yaml'
    source.write_text(yaml.safe_dump(cfg))
    generate(source, dst / 'world')
    target = dst / 'world/osp_studio.sdf'
    tree = ET.parse(target)
    world = tree.find('world')
    world.find('physics/max_step_size').text = '.004'
    # Move the simulation to the same origin convention as the board.
    for model in world.findall('model'):
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
            pose.text = '0 0 0 0 0 0'
        values = list(map(float, pose.text.split()))
        values[0] -= 4.5
        values[1] -= 3.
        pose.text = ' '.join(map(str, values))
    walls = cfg['arena']['walls']
    for index, wall in enumerate(walls):
        model = world.find(f"model[@name='arena_wall{index}']")
        assert model is not None, 'world generator ignored the requested geometry'
        size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
        assert size[:2] == [wall['sx'], wall['sy']]
    for index, marker in enumerate(cfg['targets']):
        printed = world.find(f"model[@name='target{index}']")
        visual = printed.find("link/visual[@name='white']")
        cells = [v for v in printed.findall('link/visual') if v.get('name','').startswith('cell')]
        paper_x = float(visual.findtext('pose', '0 0 0 0 0 0').split()[0])
        assert cells and all(float(v.findtext('pose').split()[0]) > paper_x for v in cells)
        model_yaw = float(printed.findtext('pose').split()[5])
        assert math.isclose(model_yaw, marker['yaw'], abs_tol=1e-9)
    tree.write(target, encoding='utf-8', xml_declaration=True)
    def shift(record):
        return dict(record, x=record['x']-4.5, y=record['y']-3.)
    translated = [shift(t) for t in cfg['targets']]
    x,y,yaw = cfg['start']
    scene = dict(seed=seed, topology=topology, room_size=[9.,6.],
                 partitions=[shift(wall) for wall in walls[4:]],
                 obstacles=[shift(item) for item in cfg['obstacles']], targets=translated,
                 robot_spawn=dict(x=x-4.5,y=y-3.,yaw=yaw))
    (dst/'scene.json').write_text(json.dumps(scene, indent=2)+'\n')
    (dst/'scene.yaml').write_text(yaml.safe_dump(dict(start=[x-4.5,y-3.,yaw],duration=600.,targets=translated)))
    return hashlib.sha256(target.read_bytes()).hexdigest()
