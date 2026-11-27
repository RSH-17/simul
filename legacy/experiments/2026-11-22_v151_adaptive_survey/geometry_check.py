"""Offline fixture QA. This clearance/line-of-sight check is not a Nav2 plan."""
import argparse
from collections import deque
import json
import math
from pathlib import Path

from fixtures import scenario


def clearance(x, y, shapes):
    distances = []
    for obj in shapes:
        dx, dy = x - obj['x'], y - obj['y']
        if obj.get('type') == 'cylinder':
            distances.append(math.hypot(dx,dy)-obj['radius'])
        else:
            # Fixtures use only axis-aligned rectangular collision geometry.
            assert obj.get('yaw',0.) == 0.
            ex, ey = abs(dx)-obj['sx']*.5, abs(dy)-obj['sy']*.5
            distances.append(math.hypot(max(0.,ex),max(0.,ey))+min(max(ex,ey),0.))
    return min(distances)


def check(seed):
    cfg, topology = scenario(seed)
    shapes = cfg['arena']['walls']+cfg['obstacles']
    grid = {(ix,iy) for ix in range(91) for iy in range(61) if clearance(ix*.1,iy*.1,shapes) >= .42}
    start = tuple(round(x/.1) for x in cfg['start'][:2])
    assert start in grid
    reached = {start}
    pending = deque([start])
    while pending:
        ix,iy = pending.popleft()
        for pt in ((ix+1,iy),(ix-1,iy),(ix,iy+1),(ix,iy-1)):
            if pt in grid and pt not in reached:
                reached.add(pt)
                pending.append(pt)
    views = {}
    for t in cfg['targets']:
        visible = []
        for ix,iy in sorted(reached):
            x,y = ix*.1,iy*.1
            dx,dy = x-t['x'],y-t['y']
            dist = math.hypot(dx,dy)
            if not .75 <= dist <= 1.3:
                continue
            cosine=(dx*math.cos(t['yaw'])+dy*math.sin(t['yaw']))/dist
            if cosine < math.cos(math.radians(35)):
                continue
            if any(clearance(t['x']+dx*i/40,t['y']+dy*i/40,shapes) <= 0 for i in range(1,41)):
                continue
            visible.append([round(x,2),round(y,2)])
        assert visible, (seed,t['id'])
        views[t['id']] = len(visible)
    return dict(seed=seed,topology=topology,connected_free_cells=len(reached),all_free_cells=len(grid),
                camera_view_cells_per_target=views,grid_m=.1,minimum_center_clearance_m=.42,
                view_range_m=[.75,1.3],maximum_incidence_deg=35,
                scope='idealized connected floor and unobstructed bearing; no runtime Nav2 path or RGB certification')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    data = json.dumps([check(112801),check(112802)], indent=2)+'\n'
    if args.out:
        args.out.write_text(data)
    else:
        print(data)
