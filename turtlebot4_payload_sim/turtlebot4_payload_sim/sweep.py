"""Generate controlled payload mass, height, and COM-offset configurations."""
import argparse
import copy
import csv
import json
from pathlib import Path

import yaml
from turtlebot4_payload_sim.model import generate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--official-xacro', type=Path, required=True)
    parser.add_argument('--base-config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(args.base_config.read_text())
    specs = [('mass_0', 0., 0., 0., 0., 'baseline', True),
             ('mass_1_0', 1., 0., 0., 0., 'mass_only', True),
             ('mass_1_5', 1.5, 0., 0., 0., 'mass_only', True),
             ('mass_2_0', 2., 0., 0., 0., 'mass_only', True),
             ('height_low', 1.5, -.05, 0., 0., 'mass_only', False),
             ('height_high', 1.5, .05, 0., 0., 'mass_only', True),
             ('offset_front', 1.5, 0., .05, 0., 'mass_only', True),
             ('offset_back', 1.5, 0., -.05, 0., 'mass_only', True),
             ('offset_left', 1.5, 0., 0., .05, 'mass_only', True),
             ('offset_right', 1.5, 0., 0., -.05, 'mass_only', True)]
    rows = []
    for name, mass, dz, dx, dy, variant, valid in specs:
        config = copy.deepcopy(base)
        config['macbook']['mass_kg'] = mass if mass else 1.5
        center = config['macbook']['center_m']
        center[0] += dx
        center[1] += dy
        center[2] += dz
        path = args.output_dir/f'{name}.yaml'
        path.write_text(yaml.safe_dump(config))
        _, report = generate(args.official_xacro, config, variant)
        rows.append({'condition': name, 'variant': variant, 'config': str(path),
                     'macbook_mass_kg': mass, 'mount_height_offset_m': dz,
                     'com_offset_x_m': dx, 'com_offset_y_m': dy,
                     'physically_valid_without_custom_mount': valid,
                     'robot_mass_kg': report['with_payload']['mass_kg'],
                     'robot_com_z_m': report['with_payload']['center_of_mass_base_link_m'][2],
                     'note': ('Lower mounting intersects the existing official plate; do not run as physical comparison'
                              if not valid else 'Rigidly fixed hypothetical mount; actual bracket not measured')})
    with (args.output_dir/'mass_properties.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir/'manifest.json').write_text(json.dumps(rows, indent=2))
    print(json.dumps({'conditions': len(rows),
                      'valid_dynamic_conditions': sum(r['physically_valid_without_custom_mount'] for r in rows)},
                     indent=2))


if __name__ == '__main__':
    main()
