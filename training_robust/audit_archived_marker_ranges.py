"""Approximate range diagnosis from compressed RGB video and saved SLAM scans.

Uses observed inputs only. Saved scans are sparse and preprocessed, and video
does not preserve the RGB acquisition stamp. This is not a detector replay or
evidence of correct marker coordinates. No ROS nodes or policy outputs exist.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from replay_marker_range_inputs import fit_range


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rgb_rectangle(width, height):
    # Same rectangle and fit_image rounding as the frozen video compositor.
    scale = min(310/width, 186/height)
    w, h = max(1, round(width*scale)), max(1, round(height*scale))
    return 12+(310-w)//2, 522+(186-h)//2, w, h


def source_pixels(corners, width, height):
    _, _, w, h = rgb_rectangle(width, height)
    return (np.asarray(corners)+.5)/[w/width, h/height]-.5


def fixed_pose(xml, link):
    edges = {j.find('child').get('link'): j for j in ET.fromstring(xml).findall('joint')}
    def pose(name, seen):
        if name == 'base_link':
            return np.eye(4)
        if name in seen:
            raise ValueError('Cyclic robot description')
        joint = edges[name]
        if joint.get('type') != 'fixed':
            raise ValueError('Sensor path is not fixed')
        local = np.eye(4)
        origin = joint.find('origin')
        if origin is not None:
            local[:3,3] = list(map(float,origin.get('xyz','0 0 0').split()))
            local[:3,:3] = Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
        return pose(joint.find('parent').get('link'), seen | {name}) @ local
    return pose(link, set())


def audit(job, output):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    output.mkdir(parents=True, exist_ok=False)
    episodes = list((job/'episodes').iterdir())
    if len(episodes) != 1:
        raise ValueError('Require one completed episode per job')
    episode = episodes[0]
    if not (episode/'evaluation.json').exists():
        raise ValueError('Only completed recordings may be audited')
    datasets = list(episode.glob('*/map.data'))
    if len(datasets) != 1:
        raise ValueError('Require exactly one saved SLAM dataset')
    inventory = json.loads((job/'input_inventory.json').read_text())
    info = inventory['streams']['info']
    if (info['width'],info['height']) != (320,240):
        raise ValueError('This audit requires the recorded official camera profile')
    source = Path(get_package_share_directory('turtlebot4_description'))/'urdf/standard/turtlebot4.urdf.xacro'
    xml = xacro.process_file(str(source), mappings={'gazebo':'ignition','namespace':''}).toxml()
    canonical = ET.canonicalize(xml_data=xml,strip_text=True,with_comments=False)
    if hashlib.sha256(canonical.encode()).hexdigest() != inventory['actual_description_sha256']:
        raise ValueError('Installed description differs from recorded native robot')
    camera = fixed_pose(xml, 'oakd_rgb_camera_optical_frame')
    scan = fixed_pose(xml, 'rplidar_link')
    transform = np.linalg.inv(scan) @ camera
    cpp = Path(__file__).with_name('export_slam_scan_readings.cpp')
    binary = output/'export_scan_readings'
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(cpp),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lkartoSlamToolbox',
        '-lboost_serialization','-lboost_thread','-lpthread','-o',str(binary)],check=True)
    with (output/'export.log').open('w') as log:
        subprocess.run([str(binary),str(datasets[0]),str(output/'scan_readings.csv')],check=True,stdout=log,stderr=log)
    scans = np.loadtxt(output/'scan_readings.csv',delimiter=',',skiprows=1,ndmin=2)
    graph = np.loadtxt(datasets[0].with_name('graph_scan_poses.csv'),delimiter=',',skiprows=1,ndmin=2)
    if scans.shape[1] != 649 or scans.shape[0] != graph.shape[0] or not np.allclose(scans[:,:2],graph[:,:2],atol=1e-9,rtol=0):
        raise ValueError('Export does not match the original saved graph nodes and timestamps')
    if not np.allclose(scans[:,6:8],scan[:2,3],atol=1e-7) or not np.allclose(scans[:,8],math.atan2(scan[1,0],scan[0,0]),atol=1e-7):
        raise ValueError('Stored scan offset differs from official sensor model')
    # 11.95 m is ambiguous: discard both adapter no-returns and any genuine
    # hits at that exact distance. Never turn a substituted endpoint into a hit.
    ambiguous = np.isclose(scans[:,9:],11.95,atol=1e-5,rtol=0)
    scans[:,9:][ambiguous] = np.inf
    K = np.asarray(info['K']).reshape(3,3)
    frames = list(csv.DictReader((episode/'video/frames.csv').open()))
    cap = cv2.VideoCapture(str(episode/'video/mission.mp4'))
    cv2.setNumThreads(1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    x,y,w,h = rgb_rectangle(info['width'],info['height'])
    rows, decodings = [], []
    index = 0
    while True:
        ok,image = cap.read()
        if not ok:
            break
        if index >= len(frames):
            raise ValueError('Video exceeds recorded frame index')
        meta = frames[index]
        stamp = float(meta['source_sim_seconds'])
        if int(meta['held_previous_frame']):
            index += 1
            continue
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(image[y:y+h,x:x+w],cv2.COLOR_BGR2GRAY),dictionary)
        if ids is not None:
            for ident,c in zip(ids.flatten(),corners):
                corners_source = source_pixels(c.reshape(4,2),info['width'],info['height'])
                pixels = float(np.linalg.norm(corners_source-np.roll(corners_source,1,axis=0),axis=1).min())
                ray = np.linalg.solve(K,[*corners_source.mean(axis=0),1.])
                decodings.append(dict(frame=index,video_sim_s=stamp,id=int(ident),source_min_edge_estimate=pixels))
                for i in np.flatnonzero(np.abs(scans[:,1]-stamp)<.25):
                    s = scans[i]
                    ranges = s[9:]
                    direction = (transform[:3,:3] @ ray)[:2]
                    bearing = math.atan2(direction[1],direction[0])
                    angles = s[4]+np.arange(len(ranges))*s[5]
                    delta = np.arctan2(np.sin(angles-bearing),np.cos(angles-bearing))
                    def fit(values, camera_cone=False):
                        return fit_range(ray,transform[:3,3],transform[:3,:3],values,
                            s[4],s[5],s[2],s[3],stamp,s[1],camera_origin_cone=camera_cone)
                    variations = []
                    for width in (.025,.03,.04,.05):
                        restricted = ranges.copy()
                        restricted[np.abs(delta)>=width] = np.inf
                        variations.append(dict(half_angle_rad=width,result=fit(restricted)))
                    rows.append(dict(frame=index,id=int(ident),video_sim_s=stamp,scan_sim_s=float(s[1]),
                        timestamp_difference_s=float(s[1]-stamp),source_min_edge_estimate=pixels,
                        original=fit(ranges),camera_origin_cone=fit(ranges,True),narrow_cones=variations))
        index += 1
    cap.release()
    if index != len(frames):
        raise ValueError('Video and recording index differ')
    native = json.loads((episode/'live_status.json').read_text())['detector']
    source_paths = [datasets[0],datasets[0].with_name('graph_scan_poses.csv'),job/'input_inventory.json',
                    episode/'video/mission.mp4',episode/'video/frames.csv',cpp,
                    job/'source_snapshot/osp_mission/record_video.py',
                    job/'source_snapshot/osp_mission/sim_mapping_scan.py']
    report = dict(evidence_type='Approximate archived RGB/SLAM-scan diagnosis; not exact production detector input',
        job=str(job),video_frames=index,decoded_frames=len({r['frame'] for r in decodings}),
        decoded_ids=dict(Counter(str(r['id']) for r in decodings)),stored_scans=len(scans),
        exported_node_stamps_match_original_graph=True,range_pairs=len(rows),rows=rows,decodings=decodings,
        scan_from_camera=transform.tolist(),base_from_camera=camera.tolist(),camera_k=info['K'],
        description_sha256=inventory['actual_description_sha256'],camera_distortion_assumed_zero=True,
        discarded_ambiguous_11_95m_values=int(ambiguous.sum()),native_detector_diagnostics=native,
        ground_truth_used=False,policy_modified=False,recognition_improvement_established=False,
        source_sha256={str(p):digest(p) for p in source_paths},
        limits=['Video RGB is resized and lossy; decoded IDs/corners may differ from the detector',
                'Video timestamps are recorder times, not exact camera acquisition timestamps',
                'SLAM retains sparse scans and preprocessing; all pairs within 0.25 s are reported',
                'Zero distortion is assumed from the official simulator camera model; D was not archived',
                'Narrow-cone sensitivity is diagnostic, not an accurate or deployable range estimator',
                'Returning a range does not create a marker recognition or correct position'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--job',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a = p.parse_args()
    r = audit(a.job.resolve(),a.output.resolve())
    print(json.dumps({k:r[k] for k in ['video_frames','decoded_frames','decoded_ids','stored_scans','range_pairs']}))
