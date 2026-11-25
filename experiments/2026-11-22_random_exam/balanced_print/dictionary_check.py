"""Exercise every Original ID using the frozen candidate's actual image decoder."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from print_stress import render


def clean_sheets(dictionary, decode):
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    patterns = [draw(dictionary, i, 35) for i in range(len(dictionary.bytesList))]
    correct = Counter()
    other = Counter()
    centers = np.array([(x,y) for y in (60,180,300,420) for x in (60,180,300,420)])
    for start in range(0, len(patterns), len(centers)):
        for turn in range(4):
            image = np.full((480,480), 245, np.uint8)
            expected = list(range(start, min(start+len(centers), len(patterns))))
            for slot, ident in enumerate(expected):
                x,y = centers[slot]
                image[y-17:y+18,x-17:x+18] = np.rot90(patterns[ident], (slot+turn) % 4)
            corners, ids = decode(image, dictionary)
            seen = set()
            if ids is not None:
                for quad, ident in zip(corners, ids.ravel()):
                    distances = np.linalg.norm(centers[:len(expected)]-quad[0].mean(axis=0), axis=1)
                    slot = int(np.argmin(distances))
                    if distances[slot] > 8 or ident != expected[slot]:
                        other['wrong_id_or_location'] += 1
                    elif slot in seen:
                        other['duplicate'] += 1
                    else:
                        seen.add(slot)
                        correct[int(ident)] += 1
    return dict(printed_side_px=35, rotations_per_id=4, total=len(patterns)*4,
                correct=sum(correct.values()), all_four_rotations=sum(v==4 for v in correct.values()),
                missed_by_id={i:4-correct[i] for i in range(len(patterns)) if correct[i]!=4},
                wrong_id_or_location=other['wrong_id_or_location'], duplicate=other['duplicate'])


def edge_frames(dictionary, decode, edge_print, seed, pixels, negative):
    counts = Counter()
    examples = defaultdict(list)
    n = 128 if negative else len(dictionary.bytesList)
    for ident in range(n):
        rng = np.random.default_rng([seed, pixels, ident, int(negative)])
        frame, center = render(rng, dictionary, ident, pixels, negative)
        target = np.array([42 if ident % 2 == 0 else 437, int(rng.integers(80,400))])
        dx,dy = np.rint(target-center).astype(int)
        frame = cv2.warpAffine(frame, np.float32([[1,0,dx],[0,1,dy]]), (480,480),
                               flags=cv2.INTER_NEAREST, borderValue=int(frame[0,0]))
        actual_center = center + [dx,dy]
        native, small = [], []
        corners, ids = decode(frame, dictionary, small, native)
        names = [] if ids is None else ids.ravel()
        matched = []
        wrong = []
        for quad, found in zip(corners, names):
            if not negative and ident == found and np.linalg.norm(quad[0].mean(axis=0)-actual_center) <= pixels/2:
                matched.append(quad[0])
            else:
                wrong.append(int(found))
        readable = any(np.linalg.norm(q-np.roll(q,1,axis=0),axis=1).min()>=18 and
                       np.all((q>=4) & (q<=475)) for q in matched)
        look = edge_print(native, corners, 480, 480, 18)
        right_look = (look is not None and not negative and look[0] == ident and
                      np.linalg.norm(np.array(look[1:3])-actual_center)<=pixels/2)
        counts.update(frames=1, correct_decode=int(bool(matched)), pass_readable_pixel_gate=int(readable),
                      wrong_named_frames=int(bool(wrong)), correct_edge_cue=int(right_look),
                      incorrect_edge_cue=int(look is not None and not right_look))
        if wrong and len(examples['wrong_decode']) < 8:
            examples['wrong_decode'].append(dict(draw=ident, proposed=wrong))
        if look is not None and not right_look and len(examples['wrong_edge_cue']) < 8:
            examples['wrong_edge_cue'].append(dict(draw=ident, proposed=look[0]))
    return dict(counts=counts, examples=examples)


def check(source, seed):
    sys.path.insert(0, str(source.resolve()))
    from osp_mission.edge_print import edge_print
    from osp_mission.marker_pair import decode
    cv2.setNumThreads(1)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    counts = {}
    for negative, label in ((False, 'original'), (True, 'nonmarker_checker')):
        counts[label] = {pixels: edge_frames(d, decode, edge_print, seed, pixels, negative)
                         for pixels in (9,14,22)}
    modules = ('marker_pair','edge_print','warped_marker','small_wall_cues')
    hashes = {m:hashlib.sha256(Path(sys.modules['osp_mission.'+m].__file__).read_bytes()).hexdigest()
              for m in modules}
    return dict(dictionary='DICT_ARUCO_ORIGINAL', ids=len(d.bytesList), seed=seed,
                cv2=cv2.__version__, source_sha256=hashes, clean=clean_sheets(d, decode),
                edge=counts, edge_pixels='ideal shortest side before rasterization',
                scope='Rendered images. No measured physical approach, turns, or temporal ID registration.')


if __name__=='__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path, help='Frozen directory containing the osp_mission package')
    p.add_argument('output', type=Path)
    p.add_argument('--seed', type=int, default=411227)
    args = p.parse_args()
    result = check(args.source, args.seed)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
