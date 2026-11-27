"""Read actual SLAM loop settings alongside the existing official input gates."""
import argparse
import json
import math
from pathlib import Path
import shlex
import subprocess

import official_local_grid_probe as grid

LOOP_KEYS = ('do_loop_closing', 'loop_match_minimum_response_coarse',
             'loop_match_minimum_response_fine', 'loop_match_maximum_variance_coarse')


def loop_gate(report, expected):
    actual = report.get('parameters', {}).get('slam_toolbox', {})
    errors = []
    for key in LOOP_KEYS:
        value, wanted = actual.get(key), expected[key]
        matched = (value is wanted if isinstance(wanted, bool) else
                   isinstance(value, (int, float)) and not isinstance(value, bool)
                   and math.isclose(value, wanted, abs_tol=1e-6))
        if not matched:
            errors.append(dict(parameter=key, expected=wanted, observed=value))
    return dict(passed=not errors, errors=errors)


def probe(domain, output):
    command = 'source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/env', 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET', 'FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
        '/usr/bin/python3', str(Path(__file__).resolve()), '--domain', str(domain), '--output', str(output)])
    subprocess.run(['/bin/bash', '-c', command], check=True, capture_output=True, text=True, timeout=85)
    return json.loads(output.read_text())


def measure(domain):
    grid.sensor.EXPECTED = {node: dict(values) for node, values in grid.sensor.EXPECTED.items()}
    grid.sensor.EXPECTED['slam_toolbox'].update({key: None for key in LOOP_KEYS})
    return grid.measure(domain)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.write_text(json.dumps(measure(a.domain), indent=2)+'\n')
