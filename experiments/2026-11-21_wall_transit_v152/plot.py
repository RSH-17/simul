"""Independent paths and wall distance; optional-look dots mark the robot position."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def plot(root, output):
    data = json.loads((root/'report.json').read_text())
    assert len(data['comparisons']) == 1
    pair = data['comparisons'][0]
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    for column, (variant, title) in enumerate((('baseline', 'Frozen v1.5.1'), ('candidate', 'v1.5.2 r2 / wall transit'))):
        row = pair[variant]
        source = root/'plot_inputs'/variant/row['run']
        scene = json.loads((source/'scene.json').read_text())
        tables = json.loads((source/'demo_layout.json').read_text())['tables']
        width, height = scene['room_size']
        west, east, south, north = -width/2, width/2, -height/2, height/2
        outline = [(west, south)]
        if scene.get('south_recess'):
            r = scene['south_recess']
            outline += [(r['x_min'], south), (r['x_min'], south-r['depth']),
                        (r['x_max'], south-r['depth']), (r['x_max'], south)]
        outline += [(east, south), (east, north), (west, north), (west, south)]
        ax = axes[0, column]
        ax.plot(*zip(*outline), color='#535b61', lw=1)
        for box in scene['partitions']+tables+scene['obstacles']:
            rect = Rectangle((box['x']-box['sx']/2, box['y']-box['sy']/2), box['sx'], box['sy'],
                color='#dbc7a5' if box in tables else '#a3aa9c', alpha=.8)
            rect.set_transform(Affine2D().rotate_around(box['x'], box['y'], box.get('yaw', 0))+ax.transData)
            ax.add_patch(rect)
        with (source/'headings.csv').open() as stream:
            points = [p for p in csv.DictReader(stream) if float(p['elapsed_sim_s']) >= 0]
        times = [float(p['elapsed_sim_s']) for p in points]
        x = [float(p['x']) for p in points]
        y = [float(p['y']) for p in points]
        ax.plot(x, y, color='#176cc0', lw=1.3, label='Independent path')
        ax.plot(x[0], y[0], 'o', color='black', mfc='white', ms=6)
        for target in scene['targets']:
            ax.plot(target['x'], target['y'], 's', color='#148364', ms=5)
            ax.annotate(str(target['id']), (target['x'], target['y']), xytext=(4, 4), textcoords='offset points')
        looks = row.get('wall_looks', {}).get('history', [])
        for i, look in enumerate(looks):
            index = min(range(len(times)), key=lambda j: abs(times[j]-look['elapsed_mission_s']))
            ax.plot(x[index], y[index], 'o', color='#d18516', ms=5, label='Transit wall look' if i == 0 else None)
        ax.set(aspect='equal', xlim=(west-.5, east+.5), ylim=(south-1, north+.5),
            title=f"{title}\n{'PASS' if row['strict_saved_map_success'] else 'FAIL'} / {row['elapsed_sim_s']:.1f}s / {row['path_length_m']:.1f}m / bumps {row['bumper_events']}")
        ax.legend(fontsize=8, loc='upper right')
        wall = next(t for t in scene['targets'] if t['attached_to'].startswith('wall_'))
        distances = [math.hypot(px-wall['x'], py-wall['y']) for px, py in zip(x, y)]
        ax = axes[1, column]
        ax.plot(times, distances, color='#176cc0', lw=1.3)
        ax.axhline(1.5, color='#b94a32', ls='--', label='1.5m criterion')
        stamp = row.get('registered_s', {}).get(str(wall['id']))
        if stamp is not None:
            ax.axvline(stamp, color='#168365', ls=':', label='Wall ID registered')
        for i, look in enumerate(looks):
            ax.axvline(look['elapsed_mission_s'], color='#d18516', alpha=.4,
                       label='Transit wall look' if i == 0 else None)
        ax.set(xlim=(0, 600), ylim=(0, max(distances)+.5), xlabel='Simulated seconds', ylabel='True wall-marker distance (m)',
            title=f"Wall ID {wall['id']} / closest {min(distances):.2f}m")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('Same world / five boxes / known failure development regression\nGround truth is evaluator-only; table rectangles are schematic top views', fontsize=13)
    fig.savefig(output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    plot(a.root, a.output)
