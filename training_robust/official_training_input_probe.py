"""Effective 600-second official training profile, including paired raw scan checks."""
import json
from pathlib import Path
import shlex
import subprocess

from official_fine_no_return_probe import configuration_gate as fine_gate
from official_tb4_probe import motion_gate


def configuration_gate(report, policy, expected_ack=1000, live_local=True,
                       expected_duration=600., expected_resolution=.025):
    if expected_resolution != .025:
        raise ValueError('This frozen input profile requires the 2.5 cm map')
    check=fine_gate(report,policy,expected_duration)
    if report.get('scan_adapter',{}).get('passed') is not True:
        check['missing'].append('paired raw/mapping scan verification')
    check['passed']=not check['missing'] and not check['mismatches']
    return check


def probe(domain, output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/env','ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
        '/usr/bin/python3',str(Path(__file__).with_name('official_fine_no_return_probe.py')),
        '--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],check=True,capture_output=True,text=True,timeout=65)
    return json.loads(output.read_text())
