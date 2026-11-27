"""Matched world-space paths, host surfaces, and independent RGB registrations."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

def render(source,out):
    rows=json.loads((source/'results.json').read_text())
    fig,axes=plt.subplots(6,2,figsize=(10,18),layout='constrained')
    for i,case in enumerate('ABCDEF'):
        for j,policy in enumerate(['mixed153','global160']):
            r=next(r for r in rows if r['case']==case and r['policy']==policy)
            root=source/'paths'/r['run'];scene=json.loads((root/'scene.json').read_text())
            tr=np.genfromtxt(root/'track.csv',delimiter=',',names=True)
            ax=axes[i,j];ax.add_patch(Rectangle((-3.94,-2.94),7.88,5.88,color='#efefeb'))
            for b in scene['obstacles']:
                ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#6e6f71'))
            ax.plot(tr['x'],tr['y'],color='#3277a5',alpha=.85,lw=1.)
            ax.scatter([tr['x'][0]],[tr['y'][0]],s=18,marker='o',color='black')
            for t in scene['targets']:
                found=t['id'] in r['ids'];x,y=t['x'],t['y']
                ax.scatter(x,y,marker='D',s=25,c='#00956b' if found else '#d95b3e')
                ax.annotate(str(t['id']),(x,y),xytext=(1,5),textcoords='offset points',size=8)
            cov=r['camera_coverage']
            tag=(f"wall {cov['wall_fraction']:.0%}, objects {cov['object_surface_fraction']:.0%}" if cov['valid'] else 'RGB evidence unavailable')
            ax.set_title(f"{case}   {policy}   ·   IDs {len(r['ids'])}/3   ·   {tag}",fontsize=10)
            ax.set_aspect('equal');ax.set_xlim(-4.2,4.2);ax.set_ylim(-3.2,3.4);ax.set_axis_off()
    fig.savefig(out,dpi=140);plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();render(a.root,a.out)
