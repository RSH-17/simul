"""Verify the effective SLAM rotational admission flag without changing a run."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

import official_tb4_probe as base

EXPECTED = {name:dict(values) for name,values in base.EXPECTED.items()}
EXPECTED['slam_toolbox']['check_min_dist_and_heading_precisely'] = True


def configuration_gate(report, policy, expected_duration=90.):
    result = base.configuration_gate(report, policy, expected_duration=expected_duration)
    key = 'check_min_dist_and_heading_precisely'
    value = report.get('parameters',{}).get('slam_toolbox',{}).get(key)
    if value is None:
        result['missing'].append('slam_toolbox/'+key)
    elif value is not True:
        result['mismatches'].append(dict(parameter='slam_toolbox/'+key,expected=True,observed=value))
    result['passed'] = not result['missing'] and not result['mismatches']
    return result


def probe(domain, output):
    command = 'source /opt/ros/jazzy/setup.bash\nexec ' + shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],check=True,capture_output=True,text=True,timeout=50)
    return json.loads(output.read_text())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    base.EXPECTED = EXPECTED
    args.output.write_text(json.dumps(base.measure(args.domain),indent=2)+'\n')
