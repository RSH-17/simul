"""Plot measured LaserScan comparisons without estimating unmeasured motion."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from turtlebot4_payload_sim.model import VARIANTS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    reports = []
    for variant in VARIANTS:
        path = args.results/variant/'summary.json'
        if path.exists():
            reports.append(json.loads(path.read_text()))
    if not reports:
        parser.error('No measured summary.json files found')
    labels = [r['variant'] for r in reports]
    values = [r['minimum_range_m'] for r in reports]
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.bar(labels, values, color=['#0b7c88' if v is not None and v >= .4 else
                                  '#d76349' for v in values])
    ax.axhline(.4, color='#777', linestyle='--', linewidth=1,
               label='40 cm diagnostic threshold')
    ax.set_ylabel('Minimum measured /scan return (m)')
    ax.set_ylim(0, max(1.1, max(v for v in values if v is not None)*1.1))
    ax.tick_params(axis='x', labelrotation=20)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.results/'scan_minimum_comparison.png', dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for variant in ('baseline', 'mass_only', 'occlusion_only', 'full', 'repositioned'):
        path = args.results/variant/'scan.csv'
        if not path.exists():
            continue
        with path.open() as handle:
            rows = list(csv.DictReader(handle))
        if rows:
            ax.plot([float(r['wall_elapsed_s']) for r in rows],
                    [float(r['minimum_range_m']) for r in rows],
                    label=variant)
    ax.set_xlabel('Elapsed wall time since variant launch (s)')
    ax.set_ylabel('Minimum measured /scan return (m)')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.results/'scan_minimum_timeseries.png', dpi=160)
    plt.close(fig)
    filter_path = args.results/'filter/raw_filtered_scan.csv'
    extra = []
    if filter_path.exists():
        with filter_path.open() as handle:
            scan = list(csv.DictReader(handle))
        near = [r for r in scan if 140 <= int(r['beam_index']) <= 180]
        fig, ax = plt.subplots(figsize=(9, 4.5))
        x = [int(r['beam_index']) for r in near]
        raw = [float(r['raw_range_m']) for r in near]
        filtered = [float(r['filtered_range_m']) if math.isfinite(float(r['filtered_range_m']))
                    else math.nan for r in near]
        ax.plot(x, raw, 'o-', ms=3, label='Raw /scan')
        ax.plot(x, filtered, 'o-', ms=3, label='Fixture filtered')
        ax.set_xlabel('LiDAR beam index (front = 160)')
        ax.set_ylabel('Measured return (m)')
        ax.set_title('A filtered near hit cannot reveal the obstacle behind it')
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.results/'filter_same_ray.png', dpi=160)
        plt.close(fig)
        extra.append('filter_same_ray.png')
    print(json.dumps({'variants': labels,
                      'figures': ['scan_minimum_comparison.png',
                                  'scan_minimum_timeseries.png', *extra]}, indent=2))


if __name__ == '__main__':
    main()
