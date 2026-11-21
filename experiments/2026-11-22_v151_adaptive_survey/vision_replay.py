"""Paired offline comparison on the *same saved JPEGs* and their exposure times.

PYTHONPATH=<candidate>/src/osp_mission python vision_replay.py <recordings> \
  --geometry ../2026-11-21_v151_track5_boxes5/camera_geometry.json --output vision_results.json
To evaluate on another host, first use --sample-tar file.tar and unpack it there.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import platform
import tarfile
import time

N_FRAMES = 5
PERIOD = 50


def rows(run):
    with (run / 'frames.csv').open() as stream:
        for row in csv.DictReader(stream):
            i = int(row['frame'])
            if (i-1) % PERIOD < N_FRAMES or ('m112601-' in run.name and 978 <= i <= 1017):
                yield row


def sample_archive(runs, archive):
    with tarfile.open(archive, 'x', dereference=True) as output:
        for run in runs:
            for name in ['scene.json', 'frames.csv', 'headings.csv'] + [
                    f"camera/{int(row['frame']):05}.jpg" for row in rows(run)]:
                output.add(run / name, arcname=f'{run.name}/{name}', recursive=False)


def project_targets(pose, scene, geometry):
    import numpy as np
    x, y, angle = pose
    dx, dy, camera_z = geometry['translation']
    x += math.cos(angle)*dx - math.sin(angle)*dy
    y += math.sin(angle)*dx + math.cos(angle)*dy
    qx, qy, qz, qw = geometry['quaternion']
    angle += math.atan2(2*(qy*qz-qx*qw), 2*(qx*qz+qy*qw))
    fx, cx, fy, cy = (geometry['k'][i] for i in [0, 2, 4, 5])
    boxes = {}
    for target in scene['targets']:
        half = target['size'] / 2
        dx, dy = -math.sin(target['yaw'])*half, math.cos(target['yaw'])*half
        pixels = []
        for sign in [-1, 1]:
            vx, vy = target['x'] + sign*dx - x, target['y'] + sign*dy - y
            forward = math.cos(angle)*vx + math.sin(angle)*vy
            if forward <= .1:
                break
            u = cx + fx*(math.sin(angle)*vx - math.cos(angle)*vy)/forward
            for z in [-half, half]:
                pixels.append((u, cy-fy*(target['z']+z-camera_z)/forward))
        if len(pixels) == 4:
            p = np.array(pixels)
            boxes[int(target['id'])] = [*p.min(axis=0), *p.max(axis=0)]
    return boxes


def analyze(runs, geometry):
    import cv2
    import numpy as np
    import osp_mission.marker_pair as marker_pair
    from osp_mission.thorough_borders import borders
    from osp_mission.small_wall_cues import anonymous_center, candidates, fragment_candidates, usable_named_quad
    cv2.setNumThreads(1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    report, focused = [], []
    for run in runs:
        scene = json.loads((run / 'scene.json').read_text())
        headings = np.genfromtxt(run/'headings.csv', delimiter=',', names=True)
        yaw = np.unwrap(headings['yaw_rad'])
        counts = dict(frames=0, conservative_negative_frames=0, baseline_negative_frames_with_hints=0,
                      candidate_negative_frames_with_hints=0, fragment_negative_proposals=0,
                      fragment_proposals=0, baseline_proposals=0, candidate_proposals=0,
                      added_outside_all_projected_tags=0, decoded_frames=0,
                      decoded_frames_with_separate_anonymous_prints=0)
        latency, mixed_search = [], []
        for row in rows(run):
            i, stamp = int(row['frame']), float(row['elapsed_sim_s'])
            image = cv2.imread(str(run/'camera'/f'{i:05}.jpg'), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'Missing frame {run.name} {i}')
            # Native first-pass quads used by the frozen and candidate versions.
            decoded, _, rejected = cv2.aruco.detectMarkers(image, dictionary)
            old = candidates(image, decoded, rejected)
            corners, _ = marker_pair.decode(image, dictionary)
            named = [q for q in corners if usable_named_quad(q, image.shape[1], image.shape[0])]
            kept = [hit for hit in old if anonymous_center(hit, named)]
            started = time.perf_counter()
            extra = fragment_candidates(image, kept, named)
            latency.append(1000*(time.perf_counter() - started))
            final = kept + extra
            if len(corners):
                started = time.perf_counter()
                separate = [hit for hit in borders(image) if anonymous_center(hit, named)]
                mixed_search.append(1000*(time.perf_counter() - started))
                counts['decoded_frames'] += 1
                counts['decoded_frames_with_separate_anonymous_prints'] += bool(separate)
            counts['frames'] += 1
            counts['baseline_proposals'] += len(old)
            counts['candidate_proposals'] += len(final)
            counts['fragment_proposals'] += len(extra)
            pose = [np.interp(stamp, headings['elapsed_sim_s'], v) for v in [headings['x'], headings['y'], yaw]]
            boxes = project_targets(pose, scene, geometry)

            def compatible(hit, box):
                u, v = hit[:2]
                return box[0]-12 <= u <= box[2]+12 and box[1]-12 <= v <= box[3]+12

            negative = all(b[2] < -64 or b[0] > image.shape[1]+64 for b in boxes.values())
            if negative:
                counts['conservative_negative_frames'] += 1
                counts['baseline_negative_frames_with_hints'] += bool(old)
                counts['candidate_negative_frames_with_hints'] += bool(final)
                counts['fragment_negative_proposals'] += len(extra)
            counts['added_outside_all_projected_tags'] += sum(
                not any(compatible(hit, box) for box in boxes.values()) for hit in extra)
            if 'm112601-' in run.name and 978 <= i <= 1017 and 7 in boxes:
                box = boxes[7]
                for label, hits in [('baseline', old), ('candidate', final)]:
                    for hit in hits:
                        if compatible(hit, box):
                            focused.append(dict(frame=i, t=round(stamp, 3), method=label,
                                                x=round(hit[0], 2), y=round(hit[1], 2),
                                                size=round(hit[2], 2)))
        report.append(dict(run=run.name, **counts, fallback_ms_p50=round(float(np.median(latency)),3),
                           fallback_ms_p95=round(float(np.percentile(latency,95)),3),
                           mixed_outline_ms_p50=round(float(np.median(mixed_search)),3) if mixed_search else None,
                           mixed_outline_ms_p95=round(float(np.percentile(mixed_search,95)),3) if mixed_search else None))
    return dict(protocol='First 5 of every 50 exposure indices plus contiguous 978..1017 on development 112601; '
                'only 112601 used to develop proposals. 112602..112605 test the frozen proposal rule.',
                camera_pixels=[480,480], opencv=cv2.__version__, host=platform.machine(), system=platform.system(),
                threads=1, comparison='Native balanced 6–<10px small hints vs the same checks with bounded partial-square proposals.',
                limitations='Saved JPEG and planar camera-pose interpolation. A compatible projection is not proof of '
                'visibility, ID, stable track, safe destination, or approach. Conservative negatives exclude '
                'frames with any GT tag within the horizontal image plus 64 px on either side. Outside-all counts '
                'use projected target rectangles expanded by 12 px. Timings separately cover the added tiny component '
                'pass on sampled frames and anonymous outline search on decoded frames, excluding ROS projection.',
                runs=report, focused_wall7=focused)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('recordings', type=Path)
    parser.add_argument('--sample-tar', type=Path)
    parser.add_argument('--geometry', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    dirs = sorted(args.recordings.glob('v151-boxes5-track-m11260?-trial?'))
    if len(dirs) != 5:
        parser.error('five original v151 track recording directories required')
    if args.sample_tar:
        sample_archive(dirs, args.sample_tar)
    else:
        if not args.geometry or not args.output:
            parser.error('--geometry and --output required')
        result = analyze(dirs, json.loads(args.geometry.read_text()))
        args.output.write_text(json.dumps(result, indent=2)+'\n')
