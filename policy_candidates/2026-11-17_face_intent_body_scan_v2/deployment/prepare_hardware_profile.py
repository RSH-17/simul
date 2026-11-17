"""Prepare the measured trial speed profile without weakening collision checks."""
import argparse
import copy
from pathlib import Path

import yaml


def prepare(config):
    result=copy.deepcopy(config)
    result['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=.25
    result['controller_server']['ros__parameters']['FollowPath']['rotate_to_heading_angular_vel']=.5
    result['velocity_smoother']['ros__parameters']['max_velocity']=[.25,0.,.5]
    result['velocity_smoother']['ros__parameters']['min_velocity']=[-.15,0.,-.5]
    result['behavior_server']['ros__parameters']['max_rotational_vel']=.5
    for key in ('collision_monitor','local_costmap','global_costmap'):
        assert result[key]==config[key]
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.write_text(yaml.safe_dump(prepare(yaml.safe_load(args.input.read_text()))))
