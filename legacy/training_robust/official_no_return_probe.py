"""Read-only verification of the simulator's SLAM-only no-return adapter."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

import official_rotation_probe as rotation

EXPECTED = {name:dict(values) for name,values in rotation.EXPECTED.items()}
EXPECTED['slam_toolbox'].update(scan_topic='scan_mapping', max_laser_range=11.9)


def configuration_gate(report, policy, expected_duration=90.):
    check = rotation.configuration_gate(report, policy, expected_duration)
    for key, expected in [('scan_topic','scan_mapping'), ('max_laser_range',11.9)]:
        observed = report.get('parameters',{}).get('slam_toolbox',{}).get(key)
        if observed is None:
            check['missing'].append('slam_toolbox/'+key)
        elif observed != expected:
            check['mismatches'].append(dict(parameter='slam_toolbox/'+key, expected=expected, observed=observed))
    check['passed'] = not check['missing'] and not check['mismatches']
    return check


def paired_scan_gate(raw, mapped):
    metadata = ('frame','stamp','range_min','range_max','angle_min','angle_max','angle_increment',
                'time_increment','scan_time','intensities')
    if any(raw[k] != mapped[k] for k in metadata):
        return dict(passed=False, reason='Scan metadata changed')
    a, b = np.asarray(raw['ranges']), np.asarray(mapped['ranges'])
    if a.shape != b.shape or not len(a) or not math.isclose(raw['range_max'],12.,abs_tol=1e-6):
        return dict(passed=False, reason='Unexpected sensor or shape')
    no_return = np.isposinf(a)
    untouched = np.array_equal(a[~no_return], b[~no_return], equal_nan=True)
    substituted = bool(np.allclose(b[no_return],11.95,rtol=0,atol=1e-6))
    return dict(passed=untouched and substituted, original_positive_infinite=int(no_return.sum()),
                other_readings_unchanged=untouched, clearing_only_substitution=substituted)


def measure(domain):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rotation.base.EXPECTED = EXPECTED
    report = rotation.base.measure(domain)
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('no_return_read_only_observer')
    messages = {'raw':{}, 'mapped':{}}
    def receive(key, m):
        stamp = (m.header.stamp.sec,m.header.stamp.nanosec)
        messages[key][stamp] = dict(frame=m.header.frame_id, stamp=stamp,
            **{k:getattr(m,k) for k in ('range_min','range_max','angle_min','angle_max','angle_increment','time_increment','scan_time')},
            ranges=list(m.ranges), intensities=list(m.intensities))
        if len(messages[key]) > 100:
            del messages[key][next(iter(messages[key]))]
    for key,topic in [('raw','scan'),('mapped','scan_mapping')]:
        node.create_subscription(LaserScan,topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data)
    deadline = time.monotonic()+10
    pairs=[]
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            common = messages['raw'].keys() & messages['mapped'].keys()
            if len(common)>=5:
                pairs=[paired_scan_gate(messages['raw'][s],messages['mapped'][s]) for s in sorted(common)]
                break
    finally:
        node.destroy_node(); rclpy.shutdown()
    report['scan_adapter'] = dict(passed=len(pairs)>=5 and all(p['passed'] for p in pairs), pairs=pairs,
        positive_infinite_observed=any(p.get('original_positive_infinite',0)>0 for p in pairs),
        raw_topic='scan', slam_topic='scan_mapping', hardware_qualified=False)
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    a.output.write_text(json.dumps(measure(a.domain),indent=2)+'\n')
