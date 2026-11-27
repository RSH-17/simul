import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from turtlebot4_payload_sim.model import generate


OFFICIAL = Path('/opt/ros/jazzy/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro')
CONFIG = Path(__file__).resolve().parents[1]/'config/payload.yaml'


@pytest.mark.skipif(not OFFICIAL.exists(), reason='Requires installed Jazzy TurtleBot4 model')
def test_payload_inertia_and_official_sensor_preservation():
    config = yaml.safe_load(CONFIG.read_text())
    baseline_xml, baseline = generate(OFFICIAL, config, 'baseline')
    full_xml, full = generate(OFFICIAL, config, 'full')
    a = ET.fromstring(baseline_xml)
    b = ET.fromstring(full_xml)
    for name in ('rplidar_link', 'rplidar_joint'):
        tag = 'link' if name.endswith('_link') else 'joint'
        original = a.find(f"{tag}[@name='{name}']")
        modified = b.find(f"{tag}[@name='{name}']")
        assert ET.tostring(original) == ET.tostring(modified)
    original_sensor = [ET.tostring(g) for g in a.findall('gazebo')
                       if g.get('reference') == 'rplidar_link']
    modified_sensor = [ET.tostring(g) for g in b.findall('gazebo')
                       if g.get('reference') == 'rplidar_link']
    assert original_sensor == modified_sensor
    mac = b.find("link[@name='payload_macbook']")
    assert float(mac.find('inertial/mass').get('value')) == 1.5
    inertia = mac.find('inertial/inertia')
    assert math.isclose(float(inertia.get('ixx')), 1.5*(.22**2+.016**2)/12)
    assert math.isclose(float(inertia.get('iyy')), 1.5*(.30**2+.016**2)/12)
    assert math.isclose(float(inertia.get('izz')), 1.5*(.30**2+.22**2)/12)
    assert math.isclose(full['with_payload']['mass_kg']-baseline['with_payload']['mass_kg'], 1.54)
    assert full['with_payload']['center_of_mass_base_link_m'][2] > baseline['with_payload']['center_of_mass_base_link_m'][2]
    before = baseline['with_payload']['inertia_about_com_base_link_kg_m2']
    after = full['with_payload']['inertia_about_com_base_link_kg_m2']
    assert all(after[i][i] > before[i][i] for i in range(3))


@pytest.mark.skipif(not OFFICIAL.exists(), reason='Requires installed Jazzy TurtleBot4 model')
def test_occlusion_only_keeps_mass_near_baseline_and_bad_size_rejected():
    config = yaml.safe_load(CONFIG.read_text())
    _, baseline = generate(OFFICIAL, config, 'baseline')
    _, occlusion = generate(OFFICIAL, config, 'occlusion_only')
    assert abs(occlusion['with_payload']['mass_kg']-baseline['with_payload']['mass_kg']) < .001
    bad = copy.deepcopy(config)
    bad['macbook']['size_m'][0] = 0
    with pytest.raises(ValueError):
        generate(OFFICIAL, bad, 'full')


@pytest.mark.skipif(not OFFICIAL.exists(), reason='Requires installed Jazzy TurtleBot4 model')
def test_repositioned_variant_moves_laptop_and_fixture():
    config = yaml.safe_load(CONFIG.read_text())
    xml, report = generate(OFFICIAL, config, 'repositioned')
    root = ET.fromstring(xml)
    joint = root.find("joint[@name='payload_macbook_fixed']/origin")
    assert [float(x) for x in joint.get('xyz').split()] == config['macbook']['repositioned_center_m']
    assert report['with_payload']['center_of_mass_base_link_m'][0] > .01
