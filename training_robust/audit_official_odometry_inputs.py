"""Freeze and evaluate a partial official TB4 odometry/IMU recording offline.

This evaluator reads files only. It never publishes ROS data or starts native
processes. The unchanged estimator receives wheel pose and gyro rate only;
independent truth is loaded afterwards to measure drift from the common start.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import audit_imu_shadow
import pose_increment_gyro


FIELDS = ['sensor', 'stamp', 'frame', 'child_frame', 'x', 'y',
          'qx', 'qy', 'qz', 'qw', 'linear_x', 'angular_z']
FRAMES = {'odom': ('odom', 'base_link'), 'imu': ('turtlebot4/imu_link/imu', '')}
DESCRIPTION = Path('/opt/ros/jazzy/share/irobot_create_description/urdf')
SNAPSHOT_FILES = {'launch.json', 'source_sha256.json', 'passivity_check.json',
    'platform_before.json', 'platform_after.json', 'inputs.csv', 'truth.csv',
    'create3.urdf.xacro', 'imu.urdf.xacro'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def complete_prefix(data):
    """Keep exactly complete newline-terminated records, including the header."""
    end = data.rfind(b'\n') + 1
    if not end:
        raise ValueError('No complete CSV header')
    return data[:end], len(data) - end


def read_prefix(path):
    with path.open('rb') as stream:
        # Bound this snapshot even while the recorder continues appending.
        stream.seek(0, 2)
        size = stream.tell()
        stream.seek(0)
        data = stream.read(size)
    if len(data) != size:
        raise ValueError('Recording was truncated during snapshot')
    prefix, omitted = complete_prefix(data)
    return prefix, dict(observed_size_bytes=size, omitted_incomplete_bytes=omitted)


def parse_inputs(data):
    reader = csv.DictReader(io.StringIO(data.decode()))
    if reader.fieldnames != FIELDS:
        raise ValueError('Unexpected sensor CSV schema')
    previous, wheel, gyro = {}, [], []
    counts = dict(odom=0, imu=0)
    duplicates = dict(odom=0, imu=0)
    for row in reader:
        if set(row) != set(FIELDS) or any(v is None for v in row.values()):
            raise ValueError('Malformed sensor CSV row')
        sensor = row['sensor']
        if sensor not in FRAMES or (row['frame'], row['child_frame']) != FRAMES[sensor]:
            raise ValueError('Unexpected sensor or frame; rotation is not assumed for other robots')
        stamp, rate = float(row['stamp']), float(row['angular_z'])
        if not np.isfinite([stamp, rate]).all() or stamp < 0:
            raise ValueError('Nonfinite or invalid acquisition timestamp/rate')
        counts[sensor] += 1
        if sensor in previous:
            old = previous[sensor]
            if stamp < float(old['stamp']):
                raise ValueError('Sensor acquisition time reversed')
            if stamp == float(old['stamp']):
                if row != old:
                    raise ValueError('Conflicting samples at one acquisition timestamp')
                duplicates[sensor] += 1
                continue
        previous[sensor] = row
        if sensor == 'odom':
            quaternion = np.array([float(row[k]) for k in ['qx', 'qy', 'qz', 'qw']])
            if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1.) > .01:
                raise ValueError('Invalid wheel orientation')
            wheel.append(audit_imu_shadow.pose(row))
        else:
            # Absolute orientation, reported velocity and truth are not inputs.
            gyro.append([stamp, rate])
    wheel = pose_increment_gyro.checked(wheel, 4)
    gyro = pose_increment_gyro.checked(gyro, 2)
    return wheel, gyro, dict(raw_counts=counts, identical_duplicates_removed=duplicates,
        unique_counts=dict(odom=len(wheel), imu=len(gyro)), frames=FRAMES,
        maximum_gap_s=dict(odom=float(np.diff(wheel[:, 0]).max()),
                           imu=float(np.diff(gyro[:, 0]).max())),
        ordering='Original per-stream order; no sorting, conflicting deduplication or reset repair')


def validate_axes(description, imu):
    """Check the installed model's fixed base->IMU rotation, not hardware calibration."""
    ns = {'xacro': 'http://ros.org/wiki/xacro'}
    uses = ET.fromstring(description).findall('.//xacro:imu_sensor', ns)
    if len(uses) != 1:
        raise ValueError('Unexpected official IMU instantiation')
    origin = uses[0].find('origin')
    if origin is None or origin.get('rpy', '0 0 0').split() != ['0', '0', '0']:
        raise ValueError('Official IMU axes are rotated')
    if uses[0].get('parent', 'base_link') != 'base_link':
        raise ValueError('Unexpected IMU parent')
    macro = ET.fromstring(imu).find('xacro:macro', ns)
    if macro is None or 'parent:=base_link' not in macro.get('params', '').split():
        raise ValueError('Unexpected IMU default parent')
    joint = macro.find('joint')
    if (joint is None or joint.get('type') != 'fixed' or joint.find('parent') is None
            or joint.find('parent').get('link') != '${parent}'):
        raise ValueError('Unexpected IMU joint')
    sensor = macro.find('./gazebo/sensor')
    if sensor is None or sensor.find('pose') is not None:
        raise ValueError('Unexpected IMU sensor pose')


def process_identity(pid):
    try:
        parts = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if parts[0] == 'Z' else parts[19]
    except FileNotFoundError:
        return None


def live_guard(recording, launch):
    if process_identity(launch['native_pid']) != launch['native_process_start']:
        raise ValueError('Original native process ended or PID was reused')
    episode = Path(launch['episode'])
    if (episode / 'evaluation.json').exists() or (recording / 'report.json').exists():
        raise ValueError('Require a live partial recording, not a completed/reused job')
    progress = json.loads((recording / 'progress.json').read_bytes())
    if (progress['status'] != 'recording' or progress['native_pid'] != launch['native_pid']
            or progress['platform_checks'] < 1 or process_identity(progress['recorder_pid']) is None):
        raise ValueError('Recorder is not actively observing the original job')
    platform = (episode / 'tb4_platform_observation.json').read_bytes()
    if json.loads(platform).get('passed') is not True:
        raise ValueError('Official platform guard is not passing')
    return progress, platform


def snapshot(recording, output):
    if output.exists():
        raise FileExistsError('Preserve earlier snapshots')
    blobs = {name: (recording / name).read_bytes() for name in
             ['launch.json', 'source_sha256.json', 'passivity_check.json']}
    launch = json.loads(blobs['launch.json'])
    for key in ['ground_truth_input', 'controls_robot', 'tf_published', 'benchmark_launched_or_restarted']:
        if launch.get(key) is not False:
            raise ValueError('Require a passive sensor-only recorder')
    passivity = json.loads(blobs['passivity_check.json'])
    if (passivity.get('passed') is not True or passivity.get('commands_or_tf_allowed') is not False
            or not set(passivity['observed_publishers']) <= {'/rosout', '/parameter_events'}):
        raise ValueError('Recorder endpoint check failed')
    frozen = json.loads(blobs['source_sha256.json'])
    if not frozen:
        raise ValueError('Missing frozen recorder source hashes')
    for path, expected in frozen.items():
        if digest(Path(path).read_bytes()) != expected:
            raise ValueError('Recorder source changed during collection')
    before, blobs['platform_before.json'] = live_guard(recording, launch)
    recorder_identity = process_identity(before['recorder_pid'])
    blobs['inputs.csv'], sensor_prefix = read_prefix(recording / 'inputs.csv')
    parse_inputs(blobs['inputs.csv'])
    blobs['truth.csv'], truth_prefix = read_prefix(Path(launch['episode']) / 'ground_truth_trajectory.csv')
    blobs['create3.urdf.xacro'] = (DESCRIPTION / 'create3.urdf.xacro').read_bytes()
    blobs['imu.urdf.xacro'] = (DESCRIPTION / 'sensors/imu.urdf.xacro').read_bytes()
    validate_axes(blobs['create3.urdf.xacro'], blobs['imu.urdf.xacro'])
    after, blobs['platform_after.json'] = live_guard(recording, launch)
    if (after['recorder_pid'] != before['recorder_pid'] or recorder_identity is None
            or process_identity(after['recorder_pid']) != recorder_identity):
        raise ValueError('Recorder process changed during snapshot')
    manifest = dict(schema_version=1, captured_utc=datetime.now(timezone.utc).isoformat(),
        evidence_type='Frozen partial recording from an ongoing native episode',
        recording=str(recording), episode=launch['episode'], native_pid=launch['native_pid'],
        native_process_start=launch['native_process_start'], domain=launch['domain'],
        recorder_process_start=recorder_identity, progress_before=before, progress_after=after,
        original_process_verified_before_and_after=True, recording_complete=False,
        sensor_prefix=sensor_prefix, truth_prefix=truth_prefix,
        file_sha256={name: digest(data) for name, data in blobs.items()})
    output.mkdir(parents=True, exist_ok=False)
    for name, data in blobs.items():
        (output / name).write_bytes(data)
    (output / 'snapshot.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def audit(recording):
    manifest = json.loads((recording / 'snapshot.json').read_bytes())
    if (manifest.get('original_process_verified_before_and_after') is not True
            or manifest.get('recording_complete') is not False):
        raise ValueError('Require a verified partial snapshot')
    if set(manifest.get('file_sha256', {})) != SNAPSHOT_FILES:
        raise ValueError('Missing snapshot file hashes')
    for name, expected in manifest['file_sha256'].items():
        if Path(name).name != name or digest((recording / name).read_bytes()) != expected:
            raise ValueError('Snapshot content changed')
    validate_axes((recording / 'create3.urdf.xacro').read_bytes(), (recording / 'imu.urdf.xacro').read_bytes())
    wheel, gyro, inputs = parse_inputs((recording / 'inputs.csv').read_bytes())
    estimated = pose_increment_gyro.integrate(wheel, gyro)
    # Construct and fingerprint the estimate BEFORE interpreting evaluator truth.
    estimate_sha256 = digest(estimated.astype('<f8').tobytes())
    truth_rows = csv.DictReader(io.StringIO((recording / 'truth.csv').read_text()))
    truth = pose_increment_gyro.checked([[float(row[k]) for k in
        ['sim_seconds', 'world_x', 'world_y', 'world_yaw']] for row in truth_rows], 4)
    truth[:, 3] = np.unwrap(truth[:, 3])
    wheel[:, 3] = np.unwrap(wheel[:, 3])
    result = audit_imu_shadow.compare(truth, dict(wheel=wheel, pose_increment_gyro=estimated))
    eligible = int(((estimated[:, 0] >= result['common_start_s']) &
                    (estimated[:, 0] < result['common_end_s'])).sum())
    sources = [Path(__file__), Path(pose_increment_gyro.__file__), Path(audit_imu_shadow.__file__),
               Path(__file__).with_name('audit_pose_trace.py')]
    result.update(snapshot=manifest, snapshot_manifest_sha256=digest((recording / 'snapshot.json').read_bytes()),
        inputs=inputs, sensor_estimate_sha256=estimate_sha256,
        truth_sampling=dict(samples=len(truth), maximum_gap_s=float(np.diff(truth[:, 0]).max()),
            interpolation_limit_s=.35, candidate_estimates_in_common_interval=eligible,
            matched_estimates=result['estimators']['pose_increment_gyro']['position_error_m']['samples']),
        source_sha256={str(p.resolve()): digest(p.read_bytes()) for p in sources},
        evidence_type='Offline sensor-only replay of partial ongoing native recording',
        estimator_parameters=dict(max_wheel_gap_s=.1, max_gyro_gap_s=.05),
        ground_truth_supplied_to_estimator=False, absolute_imu_orientation_used=False,
        reported_wheel_twist_used=False, scale_fitted=False, configuration_deployed=False,
        native_navigation_validated=False, hardware_validated=False, production_promoted=False,
        limitations=['Partial development recording; not a completed episode or unseen-scenario qualification.',
            'Each estimate has only one initial alignment; drift before the common start is excluded.',
            'Independent truth is interpolated only across gaps of at most 0.35 seconds.',
            'Installed simulation IMU axes checked; hardware extrinsics, noise and gyro bias are uncalibrated.',
            'Planar integration has no validated online TF, covariance or dropout recovery adapter.'])
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--recording', type=Path, help='Live recording to freeze in a new output directory')
    inputs.add_argument('--snapshot', type=Path, help='Existing frozen snapshot to evaluate into a new JSON output')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve earlier outputs')
    if args.snapshot:
        result = audit(args.snapshot.resolve())
        output = args.output
    else:
        snapshot(args.recording.resolve(), args.output.resolve())
        result = audit(args.output.resolve())
        output = args.output / 'evaluation.json'
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['duration_s', 'actual_path_m', 'estimators']}, indent=2))
