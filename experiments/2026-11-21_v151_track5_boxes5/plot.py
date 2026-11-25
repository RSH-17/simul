"""Independent simulator paths on the five user-requested large track layouts."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D

def plot(root,output):
    report=json.loads((root/'report.json').read_text())
    fig,axes=plt.subplots(3,2,figsize=(13,12),constrained_layout=True)
    for ax,row in zip(axes.flat,report['runs']):
        source=root/'plot_inputs'/row['run'];scene=json.loads((source/'scene.json').read_text())
        tables=json.loads((source/'demo_layout.json').read_text())['tables']
        width,height=scene['room_size'];west,east,south,north=-width/2,width/2,-height/2,height/2
        outline=[(west,south)]
        if scene.get('south_recess'):
            r=scene['south_recess'];outline += [(r['x_min'],south),(r['x_min'],south-r['depth']),(r['x_max'],south-r['depth']),(r['x_max'],south)]
        outline += [(east,south),(east,north),(west,north),(west,south)]
        ax.plot([p[0] for p in outline],[p[1] for p in outline],color='#50575f',lw=1)
        for b in scene['partitions']+tables+scene['obstacles']:
            color='#dbc7a5' if b in tables else '#9b9f94'
            rect=Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],facecolor=color,alpha=.75)
            rect.set_transform(Affine2D().rotate_around(b['x'],b['y'],b.get('yaw',0))+ax.transData);ax.add_patch(rect)
        with (source/'headings.csv').open() as stream:
            points=[r for r in csv.DictReader(stream) if float(r['elapsed_sim_s'])>=0]
        x=[float(p['x']) for p in points];y=[float(p['y']) for p in points]
        ax.plot(x,y,lw=1.3,color='#176cc0');ax.plot(x[0],y[0],'o',mfc='white',mec='black',ms=6)
        for target in scene['targets']:
            ax.plot(target['x'],target['y'],'s',color='#178868',ms=5)
            ax.annotate(str(target['id']),(target['x'],target['y']),xytext=(4,4),textcoords='offset points',fontsize=9)
        ax.set(aspect='equal',xlim=(-width/2-.5,width/2+.5),ylim=(-height/2-1,height/2+.5),
            title=f"Seed {row['seed']} | {'PASS' if row['strict_saved_map_success'] else 'FAIL'} | {row['elapsed_sim_s']:.1f}s | {row['path_length_m']:.1f}m | bumps {row['bumper_events']}")
        ax.tick_params(labelsize=8)
    axes.flat[5].axis('off')
    axes.flat[5].text(.04,.85,'Frozen v1.5.1-balanced\n480 square / 5 FPS\nIndependent simulator position\nSame demo room, 5 randomized layouts\nMarkers: 1 wall + 2 boxes\nTable shapes are schematic top views',va='top',fontsize=13)
    fig.suptitle('Large demo track: five native missions, no policy tuning between runs',fontsize=15)
    fig.savefig(output,dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(13,9),constrained_layout=True)
    for ax,row in zip(axes.flat,report['runs']):
        source=root/'plot_inputs'/row['run'];scene=json.loads((source/'scene.json').read_text())
        target=next(t for t in scene['targets'] if t['attached_to'].startswith('wall_'))
        with (source/'headings.csv').open() as stream:points=[p for p in csv.DictReader(stream) if float(p['elapsed_sim_s'])>=0]
        times=[float(p['elapsed_sim_s']) for p in points]
        distances=[math.hypot(float(p['x'])-target['x'],float(p['y'])-target['y']) for p in points]
        ax.plot(times,distances,color='#176cc0',lw=1.2)
        ax.axhline(1.5,color='#ba4b35',ls='--',lw=1,label='1.5m approach criterion')
        registered=row.get('registered_s',{}).get(str(target['id']))
        if registered is not None:ax.axvline(registered,color='#168369',ls=':',lw=1,label='ID registered')
        ax.set(xlim=(0,600),ylim=(0,max(distances)+.5),xlabel='Simulated seconds',ylabel='True distance (m)',
            title=f"Seed {row['seed']} | Wall ID {target['id']} | Minimum {min(distances):.2f}m")
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes.flat[5].axis('off');axes.flat[5].text(.05,.85,'Wall-marker access over time\nIndependent simulator position\nRed: required physical approach\nGreen: first registered ID\nNo ground truth was supplied to policy',va='top',fontsize=13)
    fig.suptitle('Did the robot approach the wall marker?',fontsize=16)
    fig.savefig(output.with_name('wall_distance.png'),dpi=160);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();plot(args.root,args.output)
