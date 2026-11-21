"""Render paired simulator trajectories with the evaluator-only fixture geometry."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle
from matplotlib.transforms import Affine2D


def panel(ax, folder, row):
    scene=json.loads((folder/'scene.json').read_text())
    layout=folder/'layout.json'
    tables=json.loads(layout.read_text())['tables'] if layout.exists() else []
    for shape,color in [(s,'#ded5c7') for s in tables]+[(s,'#aeb5b3') for s in scene['partitions']+scene['obstacles']]:
        x,y=shape['x'],shape['y']
        if shape.get('type')=='cylinder':
            patch=Circle((x,y),shape['radius'],color=color)
        else:
            sx,sy=shape['sx'],shape['sy']
            patch=Rectangle((x-sx/2,y-sy/2),sx,sy,facecolor=color)
            patch.set_transform(Affine2D().rotate_around(x,y,shape.get('yaw',0))+ax.transData)
        ax.add_patch(patch)
    w,h=[n/2 for n in scene['room_size']]
    outline=[(-w,-h)]
    recess=scene.get('south_recess')
    if recess:
        a,b,d=[recess[key] for key in ('x_min','x_max','depth')]
        outline.extend([(a,-h),(a,-h-d),(b,-h-d),(b,-h)])
    outline.extend([(w,-h),(w,h),(-w,h),(-w,-h)])
    ax.plot(*zip(*outline),lw=1,color='#414749')
    with (folder/'track.csv').open() as source:
        poses=[(float(p['x']),float(p['y'])) for p in csv.DictReader(source)]
    ax.plot(*zip(*poses),color='#266899',lw=1,alpha=.88)
    ax.plot(*poses[0],marker='o',markersize=5,markerfacecolor='white',markeredgecolor='#14191b',zorder=4)
    ax.plot(*poses[-1],marker='x',markersize=6,color='#14191b',zorder=4)
    for t in scene['targets']:
        ident=t['id']
        color='#05715a' if ident in row['verified_near_ids'] else '#c47e24' if ident in row['final_ids'] else '#853842'
        ax.plot(t['x'],t['y'],marker='s',markersize=5,color=color,zorder=4)
        ax.annotate(str(ident),(t['x'],t['y']),xytext=(4,2),textcoords='offset points',fontsize=8,color=color)
    dx=.55 if w>5 else .35
    ax.set(xlim=(-w-dx,w+dx),ylim=(min(y for _,y in outline)-dx,h+dx),aspect='equal')
    ax.tick_params(axis='both',labelsize=7,length=2)
    for spine in ax.spines.values():
        spine.set_visible(False)


def plot(export,output,group):
    data=json.loads((export/'comparison.json').read_text())
    pairs=data[group]['pairs']
    if not pairs:
        raise ValueError('This split has no measured pairs yet')
    fig,axes=plt.subplots(len(pairs),2,squeeze=False,figsize=(12,3.6*len(pairs)+.7))
    for axrow,pair in zip(axes,pairs):
        for ax,role in zip(axrow,('baseline','candidate')):
            row=pair[role]
            panel(ax,export/'tracks'/row['run'],row)
            label='Mixed v1.5.1-balanced' if role=='baseline' else 'Adaptive survey r2'
            ax.set_title(f"{label}  |  {pair['seed']}  |  {len(row['verified_near_ids'])}/3 within 1.5m  |  {row['elapsed_s']:.0f}s",fontsize=10)
    handles=[Line2D([],[],marker=m,color=c,linestyle='none',label=l,markerfacecolor='white' if m=='o' else c)
             for m,c,l in [('o','#14191b','start'),('x','#14191b','end'),('s','#05715a','registered + approached'),
                           ('s','#c47e24','registered only'),('s','#853842','unregistered')]]
    fig.legend(handles=handles,ncol=5,loc='lower center',frameon=False,fontsize=9)
    fig.tight_layout(rect=(0,.075/len(pairs)+.006,1,1))
    fig.savefig(output,dpi=150,bbox_inches='tight',facecolor='white')
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('export',type=Path)
    parser.add_argument('group',choices=('regression','heldout'));parser.add_argument('output',type=Path)
    a=parser.parse_args();plot(a.export,a.output,a.group)
