"""Plot completed measured camera selection and untouched-map validation."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(here):
    matrix=json.loads((here/'camera_matrix.json').read_text())
    final=json.loads((here/'final_validation.json').read_text())
    assert matrix['complete'] and final['complete']
    selected=final['selection']['selected']
    profile=final['selection']['profile']
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),constrained_layout=True)
    groups={g['profile']:g for g in matrix['groups']}
    profiles=('balanced','glance','detail')
    ax=axes[0]
    bars=ax.bar(range(3),[groups[p]['mean_sim_s'] for p in profiles],
                color=['#168369' if p==profile else '#9da8b3' for p in profiles])
    for bar,p in zip(bars,profiles):
        group=groups[p]
        ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+2,
                f"{group['mean_sim_s']:.1f}s\n{group['strict_saved_map_successes']}/{group['n']}",ha='center',fontsize=9)
    ax.set_xticks(range(3),['480 / 5 FPS','480 / 10 FPS','640 / 5 FPS'],fontsize=9)
    ax.set(title='Camera tuning: 2 maps per ratio',ylabel='Mean simulated seconds',ylim=(0,165))
    seeds=sorted({r['seed'] for r in final['runs']})
    x=np.arange(len(seeds));colors=('#8b949e','#176cc0')
    ax=axes[1]
    for offset,variant,label,color in zip((-.19,.19),('camera',selected),('v1.5.0','v1.5.1'),colors):
        rows=[next(r for r in final['runs'] if r['seed']==seed and r['variant']==variant) for seed in seeds]
        bars=ax.bar(x+offset,[r['elapsed_sim_s'] for r in rows],.36,label=label,color=color)
        for bar,row in zip(bars,rows):
            ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+2,
                    f"{row['elapsed_sim_s']:.1f}s"+(' FAIL' if not row['strict_saved_map_success'] else ''),ha='center',fontsize=8)
    ax.set_xticks(x,[str(s) for s in seeds],fontsize=9)
    ax.set(title=f'Untouched maps: same {profile} camera',ylabel='Simulated seconds')
    ax.set_ylim(0,max(r['elapsed_sim_s'] for r in final['runs'])*1.23);ax.legend()
    ax=axes[2]
    chosen=[r for r in final['runs'] if r['variant']==selected]
    previous=[r for r in final['runs'] if r['variant']=='camera']
    for offset,rows,label,color in zip((-.19,.19),(previous,chosen),('v1.5.0','v1.5.1'),colors):
        values=[np.mean([r['path_length_m'] for r in rows]),np.mean([r['motion']['stopped_s'] for r in rows])]
        bars=ax.bar(np.arange(2)+offset,values,.36,label=label,color=color)
        for bar,value in zip(bars,values):
            ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.3,f'{value:.1f}',ha='center',fontsize=9)
    ax.set_xticks(range(2),['Path (m)','Stopped (s)'])
    ax.set(title='Untouched-map averages',ylabel='Value (units on x-axis)');ax.legend()
    for ax in axes:ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('42 Gazebo missions overall | Strict ID + approach + RGB proof + map audit | No physical robot test',fontsize=11)
    fig.savefig(here/'final_comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':plot(Path(__file__).resolve().parent)
