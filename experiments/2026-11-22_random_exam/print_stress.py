"""Development-only image check using the installed detector on deterministic, randomized prints."""
import argparse
from collections import Counter
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np


SIZES = (7, 9, 12, 16, 22, 32)


def render(rng, dictionary, ident, pixels, negative=False):
    side = 140
    border = 21
    if negative:
        # A square frame with high-contrast interior deliberately outside the tested codebook.
        cells = np.zeros((7, 7), dtype=np.uint8)
        cells[1:-1, 1:-1] = (np.indices((5, 5)).sum(axis=0) % 2) * 255
        bits = cv2.resize(cells, (side, side), interpolation=cv2.INTER_NEAREST)
    else:
        draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
        bits = draw(dictionary, int(ident), side)
    light, dark, bg = int(rng.integers(172, 248)), int(rng.integers(8, 55)), int(rng.integers(80, 170))
    paper = np.full((side+2*border, side+2*border), light, np.uint8)
    paper[border:border+side, border:border+side] = np.where(bits == 0, dark, light)
    src = np.float32([[border, border], [border+side, border], [border+side,border+side],
                      [border, border+side]]) - .5
    square = np.float64([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
    slant = rng.uniform(-math.pi/3, math.pi/3)
    quad = square * [math.cos(slant), 1.] / (1. + .14 * np.sin(slant) * square[:, :1])
    quad *= pixels / np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1).min()
    roll = rng.uniform(-math.pi, math.pi)
    c, s = math.cos(roll), math.sin(roll)
    quad = quad @ [[c, s], [-s, c]]
    extent = int(math.ceil(1.4*np.abs(quad).max())) + 8
    local_center = extent + rng.random(2)
    factor = 4
    dest = np.float32((quad + local_center + .5) * factor - .5)
    local = cv2.warpPerspective(paper, cv2.getPerspectiveTransform(src, dest),
                                (2*extent*factor, 2*extent*factor), borderValue=bg)
    local = cv2.resize(local, (2*extent, 2*extent), interpolation=cv2.INTER_AREA)
    local = cv2.GaussianBlur(local, (5, 5), sigmaX=rng.uniform(.2, .65))
    local = np.clip(local.astype(float) + rng.normal(0., 2., local.shape), 0, 255).astype(np.uint8)
    image = np.full((480, 480), bg, np.uint8)
    left, top = rng.integers(0, 480 - 2*extent+1, 2)
    image[top:top+2*extent, left:left+2*extent] = local
    return image, local_center + [left, top]


def run(source, draws, seed):
    sys.path.insert(0, str(source.resolve()))
    from osp_mission.marker_pair import decode
    cv2.setNumThreads(1)
    rng = np.random.default_rng(seed)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    tiny_supported = 'tiny' in inspect.signature(decode).parameters
    if not 1 <= draws <= len(dictionary.bytesList):
        raise ValueError('Choose at most one sample of each ID per size')
    ids = rng.choice(len(dictionary.bytesList), draws, replace=False)
    checks = {}
    for negative in (False, True):
        by_size = {}
        for size in SIZES:
            counts = Counter()
            sequence = ids[:min(16, draws)] if negative else ids
            for ident in sequence:
                frame, center = render(rng, dictionary, ident, size, negative)
                small, tiny = [], []
                corners, names = decode(frame, dictionary, small, tiny) if tiny_supported else decode(frame, dictionary, small)
                actual = [] if names is None else list(zip(names.ravel(), corners))
                correct = [(q, np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min())
                           for found,q in actual if not negative and found==ident
                           and np.linalg.norm(q[0].mean(axis=0)-center)<=size/2]
                tiny_hit = any(not negative and item[0]==ident and np.linalg.norm(np.array(item[1:3])-center)<=size/2 for item in tiny)
                local = any(np.linalg.norm(np.array(item[:2])-center)<=max(3.,size/2) for item in small)
                counts.update(dict(frames=1, decoded=bool(correct), passes_18px_image_gate=any(
                    edge>=18 and np.all((q>=4)&(q<=475)) for q,edge in correct),
                    tiny_id_cue=tiny_hit, anonymous_center=local,
                    any_local_hint=bool(correct or tiny_hit or local),
                    wrong_named_frames=any(negative or found!=ident for found,_ in actual)
                                       or any(negative or item[0]!=ident for item in tiny)))
            by_size[size] = dict(counts)
        checks['nonmarker' if negative else 'original'] = by_size
    modules = {name:Path(sys.modules['osp_mission.'+name].__file__) for name in ('marker_pair','small_wall_cues','warped_marker')}
    return dict(seed=seed, distinct_ids=draws, cv2=cv2.__version__, size_definition='ideal shortest native side in pixels',
                source_sha256={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in modules.items()}, results=checks)


if __name__ == '__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('source',type=Path);cli.add_argument('output',type=Path)
    cli.add_argument('--draws',type=int,default=128);cli.add_argument('--seed',type=int,default=341775)
    args=cli.parse_args()
    args.output.write_text(json.dumps(run(args.source,args.draws,args.seed),indent=2)+'\n')
