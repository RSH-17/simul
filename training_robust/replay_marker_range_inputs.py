"""Explain recorded RGB/LiDAR ranging with exact production-function checks.

The optional camera-origin cone is an offline diagnostic, not a deployable
estimator. More returned ranges do not establish more accurate target poses.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S

import numpy as np


def fit_range(ray,origin,rotation,ranges,angle_min,angle_increment,range_min,range_max,
              image_stamp,scan_stamp,camera_origin_cone=False):
    if abs(image_stamp-scan_stamp)>.25:return dict(reason='scan_age',scan_age_s=image_stamp-scan_stamp)
    origin=np.asarray(origin)[:2];direction=(rotation@ray)[:2]
    if np.linalg.norm(direction)<.1:return dict(reason='horizontal_ray_degenerate')
    ranges=np.asarray(ranges);angles=angle_min+np.arange(len(ranges))*angle_increment
    points=np.column_stack((ranges*np.cos(angles),ranges*np.sin(angles)))
    ray_angle=math.atan2(direction[1],direction[0])
    if camera_origin_cone:
        relative=points-origin;angles=np.arctan2(relative[:,1],relative[:,0])
    delta=np.arctan2(np.sin(angles-ray_angle),np.cos(angles-ray_angle))
    select=(abs(delta)<.07)&np.isfinite(ranges)&(ranges>range_min)&(ranges<range_max)
    points=points[select];report=dict(points=len(points),indices=np.flatnonzero(select).tolist())
    if len(points)<4:return dict(report,reason='too_few_points')
    center=np.median(points,axis=0);_,_,v=np.linalg.svd(points-center);normal=v[-1]
    residual=float(np.max(abs((points-center)@normal)));report['max_plane_residual_m']=residual
    if residual>.06:return dict(report,reason='plane_residual')
    denom=float(normal@direction);report['ray_plane_denominator']=denom
    if abs(denom)<.05:return dict(report,reason='ray_plane_parallel')
    distance=float(normal@(center-origin)/denom);report['camera_z_m']=distance
    if not .15<distance<8:return dict(report,reason='range_limits')
    return dict(report,reason='returned_range',camera_point=(ray*distance).tolist())


def narrow_cone_ranges(ray, rotation, ranges, angle_min, angle_increment, half_angle):
    """Diagnostic mask only; keep the original fit's point/residual/range gates."""
    direction = (rotation @ ray)[:2]
    bearing = math.atan2(direction[1], direction[0])
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    delta = np.arctan2(np.sin(angles-bearing), np.cos(angles-bearing))
    restricted = np.asarray(ranges, dtype=float).copy()
    restricted[np.abs(delta) >= half_angle] = np.inf
    return restricted


def replay(inputs,workspace):
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    import cv2
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from rclpy.clock import ClockType
    from rclpy.time import Time
    from scipy.spatial.transform import Rotation
    from osp_mission.detector import Detector
    source=(workspace/'src/osp_mission/osp_mission/detector.py').resolve()
    if Path(sys.modules[Detector.__module__].__file__).resolve()!=source:
        raise ValueError('Imported detector is not from the requested frozen workspace')
    rows=[]
    for file in sorted(inputs.glob('*.npz')):
        with np.load(file) as d:
            scan=LaserScan();scan.ranges=d['scan_ranges'].tolist()
            for key in ['angle_min','angle_increment','range_min','range_max']:setattr(scan,key,float(d['scan_'+key]))
            scan.header.frame_id=str(d['scan_frame']);scan.header.stamp=Time(seconds=float(d['scan_stamp'])).to_msg()
            t=TransformStamped();xyz=d['scan_from_camera_translation'];q=d['scan_from_camera_rotation_xyzw']
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            owner=S(scan=scan,tf=S(lookup_transform=lambda *a,**kw:t));rotation=Rotation.from_quat(q).as_matrix()
            for ident,corners in zip(d['ids'],d['corners']):
                pixels=float(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1).min());uv=corners.mean(axis=0)
                point=cv2.undistortPoints(np.array([[uv]],dtype=np.float64),d['camera_k'].reshape(3,3),d['camera_d'])[0,0]
                ray=np.array([*point,1.]);args=(ray,xyz,rotation,d['scan_ranges'],float(d['scan_angle_min']),
                    float(d['scan_angle_increment']),float(d['scan_range_min']),float(d['scan_range_max']),
                    float(d['image_stamp']),float(d['scan_stamp']))
                old=fit_range(*args);alternate=fit_range(*args,camera_origin_cone=True)
                # Time.from_msg(scan.header.stamp) uses ROS_TIME, whereas the
                # Time constructor defaults to SYSTEM_TIME. Match the actual
                # detector's image-header clock even in this offline replay.
                actual=Detector.lidar_point(owner,ray,str(d['camera_frame']),
                    Time(seconds=float(d['image_stamp']),clock_type=ClockType.ROS_TIME))
                expected=old.get('camera_point')
                matched=(actual is None and expected is None) or (actual is not None and expected is not None and np.allclose(actual,expected,atol=1e-5))
                if not matched:raise AssertionError('Diagnostic diverged from production: '+str(file))
                variations=[]
                for width in (.025,.03,.04,.05):
                    restricted=narrow_cone_ranges(ray,rotation,args[3],args[4],args[5],width)
                    variations.append(dict(half_angle_rad=width,result=fit_range(*args[:3],restricted,*args[4:])))
                rows.append(dict(file=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                    stamp=float(d['image_stamp']),id=int(ident),min_edge_pixels=pixels,
                    scan_stamp=float(d['scan_stamp']),scan_age_s=float(d['image_stamp']-d['scan_stamp']),
                    map_transform_available=bool(d.get('map_transform_available','map_from_camera_translation' in d)),
                    production_would_attempt_range=pixels>=10,original=old,camera_origin_cone=alternate,
                    narrow_cones=variations,production_range_reproduced=True))
    attempted=[r for r in rows if r['production_would_attempt_range']]
    return dict(evidence_type='Recorded sensor replay; not new Gazebo or confirmed target accuracy',
        snapshots=len(list(inputs.glob('*.npz'))),rows=rows,
        production_range_checks=len(rows),native_range_evidence_available=bool(rows),
        production_attempt_reasons=dict(Counter(r['original']['reason'] for r in attempted)),
        alternate_attempt_reasons=dict(Counter(r['camera_origin_cone']['reason'] for r in attempted)),
        production_function_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        ground_truth_used=False,limits=['Capture is decimated; it is not every production frame',
            'Passive capture pairs the latest received scan; the detector callback may have received another scan',
            'No full detector or mission replay; returned range does not prove a correct marker ID or map position',
            'Camera-origin and narrow cones are diagnostic alternatives only, with the same point/residual/range gates'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['inputs','workspace','output']:p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve previous sensor replay')
    result=replay(a.inputs.resolve(),a.workspace.resolve())
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
