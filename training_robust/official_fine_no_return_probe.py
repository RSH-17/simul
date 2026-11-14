"""Read-only 2.5 cm successor profile; retain the original 5 cm probe unchanged."""
import argparse
import json
from pathlib import Path

import official_no_return_probe as sensor
import official_tb4_probe as base


def configuration_gate(report, policy, expected_duration=90.):
    check=base.configuration_gate(report,policy,expected_duration=expected_duration,expected_resolution=.025)
    observed=report.get('parameters',{}).get('slam_toolbox',{})
    for key,expected in [('scan_topic','scan_mapping'),('max_laser_range',11.9),
                         ('check_min_dist_and_heading_precisely',True)]:
        if key not in observed:
            check['missing'].append('slam_toolbox/'+key)
        elif observed[key]!=expected:
            check['mismatches'].append(dict(parameter='slam_toolbox/'+key,expected=expected,observed=observed[key]))
    check['passed']=not check['missing'] and not check['mismatches']
    return check


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    a.output.write_text(json.dumps(sensor.measure(a.domain),indent=2)+'\n')
