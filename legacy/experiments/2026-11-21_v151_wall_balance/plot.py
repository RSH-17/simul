import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def plot(export,output):
    pairs=json.loads((export/'comparison.json').read_text())['pairs']
    fig,axes=plt.subplots(len(pairs),2,figsize=(12,3.8*len(pairs)),constrained_layout=True)
    for axrow,pair in zip(axes,pairs):
        for ax,(variant,row) in zip(axrow,pair.items()):
            folder=export/'tracks'/row['run']
            scene=json.loads((folder/'scene.json').read_text())
            tables=json.loads((folder/'layout.json').read_text())['tables']
            w,h=[s/2 for s in scene['room_size']]
            outline=[(-w,-h)]
            recess=scene.get('south_recess')
            if recess:
                a,b,d=(recess[k] for k in ('x_min','x_max','depth'))
                outline.extend([(a,-h),(a,-h-d),(b,-h-d),(b,-h)])
            outline.extend([(w,-h),(w,h),(-w,h),(-w,-h)])
            ax.plot(*zip(*outline),color='#45484a',lw=1)
            for box in tables+scene['partitions']+scene['obstacles']:
                x,y=box['x'],box['y']
                rect=Rectangle((x-box['sx']/2,y-box['sy']/2),box['sx'],box['sy'],
                                    facecolor='#e2d6c3' if box in tables else '#adb3af')
                rect.set_transform(Affine2D().rotate_around(x,y,box.get('yaw',0))+ax.transData); ax.add_patch(rect)
            with (folder/'track.csv').open() as f:
                poses=list(csv.DictReader(f))
            x=[float(p['x']) for p in poses]; y=[float(p['y']) for p in poses]
            ax.plot(x,y,color='#3072a6',lw=1)
            ax.plot(x[0],y[0],'o',mfc='white',mec='black',ms=5)
            for target in scene['targets']:
                got=target['id'] in row['approached']
                ax.plot(target['x'],target['y'],marker='s',color='#098562' if got else '#ae442d',ms=6)
                ax.annotate(str(target['id']),(target['x'],target['y']),xytext=(4,2),textcoords='offset points',fontsize=8)
            v='v1.5.1' if variant=='baseline' else 'v1.5.1 wall balance'
            ax.set(title=f"{row['seed']} | {v} | {row['elapsed_sim_s']:.0f} s | Wall {row['wall_min_m']:.2f} m",
                   xlim=(-w-1,w+1),ylim=(-h-1,h+.7),aspect='equal')
            ax.set_axis_off()
    fig.suptitle('Matched five-box layouts  |  green: ID + close approach confirmed   red: missing\nIndependent trajectory in blue; table outlines are schematic')
    fig.savefig(output,dpi=150)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('export',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();plot(a.export,a.output)
