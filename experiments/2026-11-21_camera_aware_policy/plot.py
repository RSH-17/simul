"""Standalone measured-comparison charts; no inferred recognition accuracy."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(report,output):
    data=json.loads(report.read_text());groups=data['groups'];rows=data['runs']
    fig,axes=plt.subplots(1,2,figsize=(12,4.6),constrained_layout=True)
    profiles=['balanced','glance','detail'];names=['480 square / 5 FPS','480 square / 10 FPS','640 square / 5 FPS']
    ax=axes[0];x=np.arange(3);colors=['#8b949e','#176cc0']
    for offset,variant,label,color in zip((-.19,.19),('baseline','candidate'),('Frozen v1.2','Camera-aware v1.5'),colors):
        selected=[next(g for g in groups if g['split']=='development' and g['variant']==variant and g['profile']==p) for p in profiles]
        values=[r['mean_sim_s'] for r in selected];bars=ax.bar(x+offset,values,.36,label=label,color=color)
        for bar,row in zip(bars,selected):
            ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+1.5,
                f"{row['mean_sim_s']:.1f}s\n{row['strict_saved_map_successes']}/{row['n']}",ha='center',fontsize=9)
    ax.set_xticks(x,names,fontsize=9);ax.set(ylabel='Mean simulated seconds',title='Development: 2 matched maps per profile',ylim=(0,110));ax.legend()
    ax=axes[1];seeds=sorted({r['seed'] for r in rows if r['split']=='held_out'});x=np.arange(len(seeds))
    for offset,variant,label,color in zip((-.19,.19),('baseline','candidate'),('v1.2 / 480 square / 10 FPS','v1.5 / 640 square / 5 FPS'),colors):
        selected=[next(r for r in rows if r['split']=='held_out' and r['variant']==variant and r['seed']==seed) for seed in seeds]
        bars=ax.bar(x+offset,[r['elapsed_sim_s'] for r in selected],.36,label=label,color=color)
        for bar,row in zip(bars,selected):
            ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+3.,f"{row['elapsed_sim_s']:.1f}s",ha='center',fontsize=9)
    ax.set_xticks(x,[str(seed) for seed in seeds]);ax.set(ylabel='Simulated seconds',title='Unseen layouts: selected policy + camera package',ylim=(0,260));ax.legend(fontsize=8)
    for ax in axes:ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('18 Gazebo missions: 3 IDs + 1.5m approach + near RGB proof + saved map audit',fontsize=12)
    fig.savefig(output,dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('report',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();plot(args.report,args.output)
