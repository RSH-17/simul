"""Create plots from measured open-loop motion and analytic mass properties."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    root = args.results
    properties = read_csv(root/'configs/mass_properties.csv')
    summaries = {}
    for row in properties:
        path = root/'motion'/row['condition']/'summary.json'
        if path.exists():
            summaries[row['condition']] = json.loads(path.read_text())
    if not summaries:
        parser.error('No motion summaries found')
    names = [f'mass_{m}' for m in ('0','1_0','1_5','2_0')]
    available = [name for name in names if name in summaries]
    masses = [float(name[5:].replace('_','.')) for name in available]
    acceleration = [summaries[name]['max_odom_accel_mps2'] for name in available]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(masses, acceleration, 'o-')
    ax.set(xlabel='Added MacBook mass (kg)', ylabel='Peak odom acceleration (m/s²)',
           title='Measured open-loop response; one trial per condition')
    fig.tight_layout()
    fig.savefig(root/'mass_vs_acceleration.png', dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name in ('mass_0','mass_1_5'):
        path = root/'motion'/name/'motion.csv'
        if not path.exists():
            continue
        rows = read_csv(path)
        if not rows:
            continue
        t0 = float(rows[0]['sim_stamp_s'])
        ax.plot([float(r['sim_stamp_s'])-t0 for r in rows],
                [float(r['speed_mps']) for r in rows], label=name)
    ax.set(xlabel='Elapsed simulated time (s)', ylabel='Wheel odom speed (m/s)',
           title='Same 0.2 m/s command, 2 s drive + 1 s stop')
    ax.legend()
    fig.tight_layout()
    fig.savefig(root/'baseline_payload_velocity.png', dpi=160)
    plt.close(fig)
    height_names = [n for n in ('mass_1_5','height_high') if n in summaries]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = list(range(len(height_names)))
    ax.plot(x, [summaries[n]['max_abs_roll_rad'] for n in height_names],
            'o-', label='|roll| max')
    ax.plot(x, [summaries[n]['max_abs_pitch_rad'] for n in height_names],
            'o-', label='|pitch| max')
    ax.set_xticks(x, ['nominal 0.357 m','high 0.407 m'][:len(height_names)])
    ax.set(ylabel='Maximum angle (rad)',
           title='Lower position excluded: it intersects the official plate')
    ax.legend()
    fig.tight_layout()
    fig.savefig(root/'height_roll_pitch.png', dpi=160)
    plt.close(fig)
    rows = [row for row in properties if row['condition'] in summaries]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.bar([row['condition'] for row in rows],
           [summaries[row['condition']]['stopping_distance_m'] or 0. for row in rows])
    ax.tick_params(axis='x', labelrotation=30)
    ax.set(ylabel='Distance after stop command (m)',
           title='Measured open-loop stopping distance; one trial each')
    fig.tight_layout()
    fig.savefig(root/'stopping_distance.png', dpi=160)
    plt.close(fig)
    turns = {}
    for row in properties:
        path = root/'turn'/row['condition']/'summary.json'
        if path.exists():
            turns[row['condition']] = json.loads(path.read_text())
    extra = []
    if turns:
        turn_names = [row['condition'] for row in properties if row['condition'] in turns]
        fig, ax = plt.subplots(figsize=(10, 4.5))
        ax.bar(turn_names, [turns[name]['yaw_change_rad'] or 0. for name in turn_names])
        ax.tick_params(axis='x', labelrotation=30)
        ax.set(ylabel='Yaw change (rad)',
               title='Same 0.6 rad/s turn command; one trial each')
        fig.tight_layout()
        fig.savefig(root/'turn_response.png', dpi=160)
        plt.close(fig)
        extra.append('turn_response.png')
    print(json.dumps({'measured_conditions': sorted(summaries),
                      'figures': ['mass_vs_acceleration.png','baseline_payload_velocity.png',
                                  'height_roll_pitch.png','stopping_distance.png',
                                  *extra]}, indent=2))


if __name__ == '__main__':
    main()
