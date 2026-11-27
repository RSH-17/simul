import csv
import io
import json

import numpy as np
import pytest

import audit_official_odometry_inputs as audit


def sensor_rows():
    rows = []
    for t in np.arange(0., 3.01, .01):
        rate, speed = .8, .2
        rows.append(dict(zip(audit.FIELDS, ['odom', str(t), 'odom', 'base_link',
            str(speed / rate * np.sin(rate * t)), str(speed / rate * (1 - np.cos(rate * t))),
            '0', '0', str(np.sin(rate * t / 2)), str(np.cos(rate * t / 2)), '999', '-999'])))
        rows.append(dict(zip(audit.FIELDS, ['imu', str(t), 'turtlebot4/imu_link/imu',
            '', '', '', '0', '0', '0', '1', '', '.7'])))
    return rows


def csv_bytes(rows):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=audit.FIELDS)
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode()


def estimate(rows):
    wheel, gyro, evidence = audit.parse_inputs(csv_bytes(rows))
    return audit.pose_increment_gyro.integrate(wheel, gyro), evidence


def test_complete_prefix_never_invents_a_partial_measurement():
    original = csv_bytes(sensor_rows())
    complete, omitted = audit.complete_prefix(original + b'odom,123')
    assert complete == original and omitted == 8
    with pytest.raises(ValueError, match='header'):
        audit.complete_prefix(b'sensor,stamp')


def test_identical_duplicate_is_reported_without_changing_motion():
    rows = sensor_rows()
    expected, _ = estimate(rows)
    rows.insert(51, rows[50].copy())
    actual, evidence = estimate(rows)
    assert np.array_equal(actual, expected)
    assert evidence['identical_duplicates_removed'] == dict(odom=1, imu=0)


def test_conflicting_duplicate_is_rejected_instead_of_choosing_a_pose():
    rows = sensor_rows()
    conflicting = dict(rows[50], x='100')
    rows.insert(51, conflicting)
    with pytest.raises(ValueError, match='Conflicting'):
        estimate(rows)


@pytest.mark.parametrize('sensor', ['odom', 'imu'])
def test_clock_reset_is_not_hidden_by_sorting(sensor):
    rows = sensor_rows()
    row = next(r for r in rows if r['sensor'] == sensor and float(r['stamp']) >= 1.)
    row['stamp'] = '0.1'
    with pytest.raises(ValueError, match='reversed'):
        estimate(rows)


@pytest.mark.parametrize('sensor, message', [('odom', 'Wheel data gap'), ('imu', 'Stale gyro')])
def test_missing_sensor_interval_is_not_bridged(sensor, message):
    rows = [r for r in sensor_rows() if r['sensor'] != sensor or not .5 < float(r['stamp']) < 1.]
    with pytest.raises(ValueError, match=message):
        estimate(rows)


def test_changed_frame_is_not_treated_as_base_aligned():
    rows = sensor_rows()
    rows[1]['frame'] = 'rotated_imu'
    with pytest.raises(ValueError, match='frame'):
        estimate(rows)


def test_reported_velocity_and_absolute_imu_orientation_are_unused():
    rows = sensor_rows()
    original, _ = estimate(rows)
    for row in rows:
        if row['sensor'] == 'imu':
            row.update(qx='0.5', qy='0.5', qz='0.5', qw='0.5')
        else:
            row.update(linear_x='-2000', angular_z='3000')
    changed, _ = estimate(rows)
    assert np.array_equal(original, changed)


@pytest.fixture
def recording(tmp_path, monkeypatch):
    source = tmp_path / 'capture.py'
    source.write_text('passive recorder fixture')
    rec = tmp_path / 'rec'
    rec.mkdir()
    episode = tmp_path / 'episode'
    episode.mkdir()
    description = tmp_path / 'description'
    (description / 'sensors').mkdir(parents=True)
    (description / 'create3.urdf.xacro').write_text('''<robot xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:imu_sensor><origin xyz="0 0 0"/></xacro:imu_sensor></robot>''')
    (description / 'sensors/imu.urdf.xacro').write_text('''<robot xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:macro params="parent:=base_link"><joint type="fixed"><parent link="${parent}"/></joint>
      <gazebo><sensor type="imu"/></gazebo></xacro:macro></robot>''')
    monkeypatch.setattr(audit, 'DESCRIPTION', description)
    monkeypatch.setattr(audit, 'process_identity', lambda pid: {10: '123', 20: '456'}.get(pid))
    files = {
        rec / 'launch.json': dict(native_pid=10, native_process_start='123', episode=str(episode), domain=230,
            ground_truth_input=False, controls_robot=False, tf_published=False, benchmark_launched_or_restarted=False),
        rec / 'progress.json': dict(native_pid=10, recorder_pid=20, platform_checks=2, status='recording'),
        rec / 'passivity_check.json': dict(passed=True, observed_publishers=['/rosout'], commands_or_tf_allowed=False),
        rec / 'source_sha256.json': {str(source): audit.digest(source.read_bytes())},
        episode / 'tb4_platform_observation.json': dict(passed=True),
    }
    for path, data in files.items():
        path.write_text(json.dumps(data))
    (rec / 'inputs.csv').write_bytes(csv_bytes(sensor_rows()))
    with (episode / 'ground_truth_trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sim_seconds', 'world_x', 'world_y', 'world_yaw'])
        for t in np.arange(0, 3.01, .01):
            writer.writerow([t, .2 / .7 * np.sin(.7 * t), .2 / .7 * (1 - np.cos(.7 * t)), .7 * t])
    return rec, tmp_path / 'snapshot'


def test_snapshot_is_frozen_partial_and_estimator_cannot_depend_on_truth(recording):
    rec, output = recording
    audit.snapshot(rec, output)
    result = audit.audit(output)
    assert result['snapshot']['recording_complete'] is False
    assert result['estimators']['pose_increment_gyro']['position_error_m']['maximum'] < 1e-12
    assert result['estimators']['wheel']['position_error_m']['maximum'] > .06
    with pytest.raises(FileExistsError):
        audit.snapshot(rec, output)
    # Different evaluator truth in an independently captured second snapshot
    # changes the error but cannot change the sensor-only trajectory.
    truth = rec.parent / 'episode/ground_truth_trajectory.csv'
    rows = list(csv.reader(io.StringIO(truth.read_text())))
    for row in rows[1:]:
        row[1] = str(float(row[1]) + float(row[0]) * .3)
    with truth.open('w') as stream:
        csv.writer(stream).writerows(rows)
    second = output.with_name('snapshot_changed_truth')
    audit.snapshot(rec, second)
    different = audit.audit(second)
    assert different['sensor_estimate_sha256'] == result['sensor_estimate_sha256']
    assert different['estimators']['pose_increment_gyro']['position_error_m']['maximum'] > .8


def test_snapshot_rejects_changed_bytes(recording):
    rec, output = recording
    audit.snapshot(rec, output)
    with (output / 'inputs.csv').open('ab') as stream:
        stream.write(b'\n')
    with pytest.raises(ValueError, match='Snapshot content changed'):
        audit.audit(output)


def test_process_reuse_mid_snapshot_is_rejected(recording, monkeypatch):
    rec, output = recording
    read = audit.read_prefix

    def replace_identity(path):
        result = read(path)
        monkeypatch.setattr(audit, 'process_identity', lambda pid: 'different_start_time')
        return result

    monkeypatch.setattr(audit, 'read_prefix', replace_identity)
    with pytest.raises(ValueError, match='PID was reused'):
        audit.snapshot(rec, output)
    assert not output.exists()


def test_no_snapshot_when_passive_source_changed(recording):
    rec, output = recording
    source = rec.parent / 'capture.py'
    source.write_text('changed')
    with pytest.raises(ValueError, match='source changed'):
        audit.snapshot(rec, output)
    assert not output.exists()


def test_failed_official_platform_guard_is_not_ignored(recording):
    rec, output = recording
    (rec.parent / 'episode/tb4_platform_observation.json').write_text('{"passed": false}')
    with pytest.raises(ValueError, match='platform guard'):
        audit.snapshot(rec, output)
    assert not output.exists()


def test_missing_truth_hash_cannot_silently_disable_integrity(recording):
    rec, output = recording
    audit.snapshot(rec, output)
    path = output / 'snapshot.json'
    manifest = json.loads(path.read_text())
    del manifest['file_sha256']['truth.csv']
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='Missing snapshot file hashes'):
        audit.audit(output)


def test_rotated_official_imu_is_rejected(recording):
    rec, output = recording
    source = audit.DESCRIPTION / 'create3.urdf.xacro'
    source.write_text(source.read_text().replace('xyz="0 0 0"', 'xyz="0 0 0" rpy="1 0 0"'))
    with pytest.raises(ValueError, match='axes are rotated'):
        audit.snapshot(rec, output)
