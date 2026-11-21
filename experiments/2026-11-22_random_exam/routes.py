"""Plot paired measured paths over the same drawn Gazebo world."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon
import numpy as np

from geometry import polygon, wall_segments


def read(path):
    return json.loads(path.read_text())


def draw(ax, world, episode, recordings, role):
    fixture=world/'run/simulation'
    scene=read(world/'scene.json')
    tables=read(fixture/'demo_layout.json')['tables']
    for name,a,b in wall_segments(scene):
        if not name.startswith('structure_'):
            ax.plot([a[0],b[0]],[a[1],b[1]],color='#64748b',lw=1.5)
    for items,color in ((scene['partitions'],'#8d99a6'),(tables,'#ddd4c3'),(scene['obstacles'],'#8495ad')):
        for body in items:
            ax.add_patch(Polygon(polygon(body),facecolor=color,edgecolor='white',lw=.5))
    path=recordings/episode['name']/'headings.csv'
    with path.open() as stream:
        samples=[(float(r['elapsed_sim_s']),float(r['x']),float(r['y'])) for r in csv.DictReader(stream)]
    samples=np.asarray(samples); samples=samples[(samples[:,0]>=0)&(samples[:,0]<=600)]
    ax.plot(samples[:,1],samples[:,2],color='#2461a1' if role=='baseline' else '#7a3e98',lw=1.3,alpha=.9)
    ax.scatter(*samples[0,1:],s=30,facecolors='white',edgecolors='#202e3e',zorder=6)
    for target in scene['targets']:
        ident=str(target['id'])
        record=episode['score']['records'][ident]
        color='#147847' if record['verified_s'] is not None else ('#bf7719' if record['registered_s'] is not None else '#a63232')
        ax.scatter(target['x'],target['y'],s=25,c=color,marker='s',zorder=7,edgecolors='white',linewidths=.5)
        offset=-11 if target['y']>3.5 else 4
        ax.annotate(ident,(target['x'],target['y']),xytext=(3,offset),textcoords='offset points',fontsize=8,color=color,
                    bbox=dict(facecolor='white',edgecolor='none',pad=.2,alpha=.8),zorder=8)
    score=episode['score']
    ax.set_title(f"{score['verified']}/{score['expected']} near   ·   {score['elapsed_s']:.0f} s   ·   {score['travel_m']:.1f} m",fontsize=9)
    ax.set_aspect('equal');ax.set_xticks([]);ax.set_yticks([])
    for side in ax.spines.values():side.set_visible(False)


def plot(root,recordings,destination):
    comparison=read(root/'comparison.json')
    pairs=comparison['pairs']
    if not pairs:raise ValueError('A completed pair is required to plot actual routes')
    fig,axes=plt.subplots(len(pairs),2,squeeze=False,figsize=(12,3.6*len(pairs)+1.1),layout='constrained')
    fig.suptitle('8780 randomized exam  |  Balanced (left) and R5 (right)',size=15)
    for i,pair in enumerate(pairs):
        world=root/'fixtures'/f"map-{pair['seed']}"
        for col,role in enumerate(('baseline','candidate')):
            draw(axes[i,col],world,pair[role],recordings,role)
        suffix=' (repeat)' if pair['repeat'] else ''
        axes[i,0].set_ylabel(f"#{pair['pair']} {pair['seed']}{suffix}",fontsize=8)
    handles=[Patch(color='#147847',label='Verified approach'),Patch(color='#bf7719',label='Registered only'),
             Patch(color='#a63232',label='Unregistered'),Line2D([],[],color='#202e3e',marker='o',
                   markerfacecolor='white',lw=0,label='Measured start')]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,-.025),ncol=4,frameon=False,fontsize=9)
    fig.savefig(destination,dpi=145,bbox_inches='tight');plt.close(fig)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    for name in ('exam','recordings','output'):cli.add_argument(name,type=Path)
    args=cli.parse_args();plot(args.exam,args.recordings,args.output)
