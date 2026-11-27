"""Flatten measured run summaries into CSV tables for audit and plotting."""
import argparse
import csv
import json
from pathlib import Path


def export(paths, destination, prefix):
    records = []
    for path in sorted(paths):
        data = json.loads(path.read_text())
        row = {**prefix(path), **{
            key: json.dumps(value) if isinstance(value, (dict, list)) else value
            for key, value in data.items()}}
        records.append(row)
    if records:
        fields = list(dict.fromkeys(key for row in records for key in row))
        with destination.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
    return len(records)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot', type=Path)
    args = parser.parse_args()
    root = args.snapshot
    counts = {}
    counts['scan'] = export(
        [path for path in [*root.glob('*/summary.json'),
                           *root.glob('front_fixture/*/summary.json')]
         if path.parent.name != 'filter'],
        root/'scan_conditions.csv',
        lambda path: {'layout': 'front' if 'front_fixture' in path.parts else 'side',
                      'condition': path.parent.name})
    counts['navigation'] = export(
        [*root.glob('navigation/*/summary.json'),
         *root.glob('front_fixture/navigation/*/summary.json')],
        root/'navigation_conditions.csv',
        lambda path: {'layout': 'front' if 'front_fixture' in path.parts else 'side',
                      'condition': path.parent.name})
    counts['straight'] = export(
        root.glob('sweep/motion/*/summary.json'),
        root/'straight_motion_conditions.csv',
        lambda path: {'condition': path.parent.name})
    counts['turn'] = export(
        root.glob('sweep/turn/*/summary.json'),
        root/'turn_motion_conditions.csv',
        lambda path: {'condition': path.parent.name})
    print(json.dumps(counts, indent=2))


if __name__ == '__main__':
    main()
