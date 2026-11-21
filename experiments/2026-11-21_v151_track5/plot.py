"""Independent simulator paths on the five user-requested large track layouts."""
import argparse
import csv
import json
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
        width,height=scene['room_size'];ax.add_patch(Rectangle((-width/2,-height/2),width,height,fill=False,color='#50575f'))
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

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();plot(args.root,args.output)
