"""Extend the installed official TurtleBot4 Standard URDF with mounted payloads."""
import argparse
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


VARIANTS = ('baseline', 'mass_only', 'occlusion_only', 'adapter_only',
            'cable_only', 'full', 'repositioned')


def vec(values, name):
    result = [float(v) for v in values]
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must be three finite numbers')
    return result


def fmt(values):
    return ' '.join(str(v) for v in values)


def add_body(root, name, mass, shape, size, center, rpy, rgba):
    if mass <= 0 or not all(v > 0 for v in size):
        raise ValueError(f'{name}: mass and size must be positive')
    link = ET.SubElement(root, 'link', name=name)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value=str(mass))
    if shape == 'box':
        x, y, z = size
        diag = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12,
                mass*(x*x+y*y)/12]
    else:
        radius, length = size
        diag = [mass*(3*radius*radius+length*length)/12]*2 + [mass*radius*radius/2]
    ET.SubElement(inertial, 'inertia', ixx=str(diag[0]), ixy='0', ixz='0',
                  iyy=str(diag[1]), iyz='0', izz=str(diag[2]))
    for kind in ('visual', 'collision'):
        element = ET.SubElement(link, kind)
        geometry = ET.SubElement(element, 'geometry')
        if shape == 'box':
            ET.SubElement(geometry, 'box', size=fmt(size))
        else:
            ET.SubElement(geometry, 'cylinder', radius=str(size[0]), length=str(size[1]))
        if kind == 'visual':
            material = ET.SubElement(element, 'material', name=f'{name}_material')
            ET.SubElement(material, 'color', rgba=rgba)
    joint = ET.SubElement(root, 'joint', name=f'{name}_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link=name)
    ET.SubElement(joint, 'origin', xyz=fmt(center), rpy=fmt(rpy))
    # The link must stay distinct in Gazebo for physical and ray-sensor auditing.
    gazebo = ET.SubElement(root, 'gazebo', reference=f'{name}_fixed')
    ET.SubElement(gazebo, 'preserveFixedJoint').text = 'true'
    return {'name': name, 'mass_kg': mass, 'center_base_link_m': center,
            'principal_inertia_kg_m2': diag}


def add_cable(root, config, parked, mass_scale):
    points = [vec(p, 'cable point') for p in config[
        'parked_points_m' if parked else 'points_m']]
    segments = []
    for a, b in zip(points, points[1:]):
        direction = [b[i]-a[i] for i in range(3)]
        length = math.sqrt(sum(d*d for d in direction))
        if length <= 0:
            raise ValueError('Cable segment has zero length')
        yaw = math.atan2(direction[1], direction[0])
        pitch = math.atan2(math.hypot(*direction[:2]), direction[2])
        segments.append((length, [(a[i]+b[i])/2 for i in range(3)],
                         [0., pitch, yaw]))
    total_length = sum(s[0] for s in segments)
    return [add_body(root, f'payload_cable_{i}',
                     float(config['mass_kg'])*mass_scale*length/total_length,
                     'cylinder', [float(config['diameter_m'])/2, length],
                     center, rpy, '0.08 0.08 0.08 1')
            for i, (length, center, rpy) in enumerate(segments)]


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mv(a, v):
    return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]


def mass_properties(root):
    """Zero-joint-position mass, COM and inertia in base_link."""
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = vec(origin.get('xyz', '0 0 0').split(), 'joint origin') if origin is not None else [0.]*3
        rpy = vec(origin.get('rpy', '0 0 0').split(), 'joint orientation') if origin is not None else [0.]*3
        joints[joint.find('child').get('link')] = (joint.find('parent').get('link'), xyz, rpy)
    identity = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    cache = {'base_link': ([0.,0.,0.], identity)}

    def pose(name):
        if name in cache:
            return cache[name]
        parent, xyz, rpy = joints[name]
        parent_pos, parent_rot = pose(parent)
        relative = mv(parent_rot, xyz)
        result = ([parent_pos[i]+relative[i] for i in range(3)],
                  matmul(parent_rot, rotation(rpy)))
        cache[name] = result
        return result

    total = 0.
    weighted = [0.,0.,0.]
    bodies = []
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        origin = inertial.find('origin')
        local = vec(origin.get('xyz', '0 0 0').split(), 'inertial origin') if origin is not None else [0.]*3
        inertial_rpy = vec(origin.get('rpy', '0 0 0').split(), 'inertial orientation') if origin is not None else [0.]*3
        pos, rot = pose(link.get('name'))
        local = mv(rot, local)
        center = [pos[i]+local[i] for i in range(3)]
        frame_rot = matmul(rot, rotation(inertial_rpy))
        tensor = inertial.find('inertia')
        if tensor is not None:
            own = [[float(tensor.get('ixx')), float(tensor.get('ixy')), float(tensor.get('ixz'))],
                   [float(tensor.get('ixy')), float(tensor.get('iyy')), float(tensor.get('iyz'))],
                   [float(tensor.get('ixz')), float(tensor.get('iyz')), float(tensor.get('izz'))]]
            rotated = matmul(matmul(frame_rot, own),
                             [[frame_rot[j][i] for j in range(3)] for i in range(3)])
            bodies.append((mass, center, rotated))
        total += mass
        for i in range(3):
            weighted[i] += mass*center[i]
    com = [x/total for x in weighted]
    inertia = [[0.]*3 for _ in range(3)]
    for mass, center, own in bodies:
        delta = [center[i]-com[i] for i in range(3)]
        radius_squared = sum(d*d for d in delta)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += own[i][j] + mass*((radius_squared if i == j else 0.)
                                                   - delta[i]*delta[j])
    return {'mass_kg': total, 'center_of_mass_base_link_m': com,
            'inertia_about_com_base_link_kg_m2': inertia}


def generate(official_xacro, config, variant):
    if variant not in VARIANTS:
        raise ValueError(f'Unknown variant: {variant}')
    original = ET.fromstring(subprocess.check_output(
        ['xacro', str(official_xacro), 'gazebo:=ignition'], text=True))
    baseline = mass_properties(original)
    root = ET.fromstring(ET.tostring(original))
    added = []
    if variant in ('mass_only', 'full', 'repositioned'):
        c = config['macbook']
        added.append(add_body(root, 'payload_macbook', float(c['mass_kg']), 'box',
                              vec(c['size_m'], 'MacBook size'),
                              vec(c['repositioned_center_m'] if variant == 'repositioned'
                                  else c['center_m'], 'MacBook center'),
                              vec(c['rpy_rad'], 'MacBook orientation'),
                              '0.4 0.4 0.45 1'))
    if variant in ('occlusion_only', 'adapter_only', 'cable_only', 'full', 'repositioned'):
        parked = variant == 'repositioned'
        scale = 0.001 if variant == 'occlusion_only' else 1.
        c = config['adapter']
        if variant != 'cable_only':
            added.append(add_body(root, 'payload_adapter', float(c['mass_kg'])*scale,
                                  'box', vec(c['size_m'], 'adapter size'),
                                  vec(c['parked_center_m' if parked else 'center_m'],
                                      'adapter center'),
                                  vec(c['rpy_rad'], 'adapter orientation'),
                                  '0.05 0.05 0.05 1'))
        if variant != 'adapter_only':
            added.extend(add_cable(root, config['cable'], parked, scale))
    report = {'variant': variant, 'baseline': baseline,
              'with_payload': mass_properties(root), 'added_links': added,
              'limitations': ['Actual payload dimensions and positions are unmeasured',
                              'Ray detection and dynamic behavior require Gazebo measurements']}
    return ET.tostring(root, encoding='unicode'), report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--official-xacro', type=Path, required=True)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1]/'config/payload.yaml')
    parser.add_argument('--variant', choices=VARIANTS, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    xml, report = generate(args.official_xacro,
                           yaml.safe_load(args.config.read_text()), args.variant)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(xml)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
