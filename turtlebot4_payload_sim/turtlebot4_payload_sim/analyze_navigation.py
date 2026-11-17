"""Plot recorded Nav2 outcomes and local costmap counts; no pooled success claim."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('navigation_results', type=Path)
    args = parser.parse_args()
    records = []
    for path in sorted(args.navigation_results.glob('*/summary.json')):
        summary = json.loads(path.read_text())
        records.append((path.parent.name, summary))
    if not records:
        parser.error('No navigation summaries found')

    labels = [name for name, _ in records]
    x = list(range(len(records)))
    fig, ax = plt.subplots(figsize=(max(8, len(records) * 1.5), 4.5))
    inflated = [r.get('costmap_inflated') or 0 for _, r in records]
    lethal = [r.get('costmap_lethal') or 0 for _, r in records]
    ax.bar(x, inflated, label='Inflated costmap cells')
    ax.bar(x, lethal, bottom=inflated, label='Lethal costmap cells')
    ax.set_xticks(x, labels, rotation=25, ha='right')
    ax.set(ylabel='Cells in last received local costmap',
           title='Local costmap snapshot at end of each run')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.navigation_results/'local_costmap_comparison.png', dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(8, len(records) * 1.5), 4.5))
    colors = ['#348a65' if r.get('goal_status') == 4 else
              '#d6aa4b' if r.get('first_motion_wall_s') is not None else
              '#cb7654' for _, r in records]
    ax.bar(x, [1] * len(records), color=colors)
    for index, (_, r) in enumerate(records):
        label = ('succeeded' if r.get('goal_status') == 4 else
                 'moved; timeout' if r.get('first_motion_wall_s') is not None else
                 'no motion; timeout')
        ax.text(index, .5, label, ha='center', va='center', rotation=90, fontsize=8)
    ax.set_xticks(x, labels, rotation=25, ha='right')
    ax.set(ylim=(0, 1), yticks=[], title='Observed Nav2 result (one trial per condition)')
    ax.legend(handles=[Patch(facecolor='#348a65', label='Goal succeeded'),
                       Patch(facecolor='#d6aa4b', label='Moved, then timed out'),
                       Patch(facecolor='#cb7654', label='No motion, timed out')],
              loc='upper center', ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.navigation_results/'navigation_trial_outcomes.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'conditions': labels, 'trials_per_condition': 1,
                      'figures': ['local_costmap_comparison.png',
                                  'navigation_trial_outcomes.png']}, indent=2))


if __name__ == '__main__':
    main()
