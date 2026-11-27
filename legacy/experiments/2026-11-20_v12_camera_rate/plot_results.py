"""Standalone scientific plots from measured replay and native results."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(root,output):
    reports=[json.loads((root/name).read_text()) for name in ('replay-map-1.json','replay-map-2.json')]
    fig,axes=plt.subplots(1,2,figsize=(12,4.9),constrained_layout=True)
    sizes=[250,320,400,480,640];rates=[5,10,15]
    grid=np.zeros((5,3))
    for i,size in enumerate(sizes):
        for j,fps in enumerate(rates):
            grid[i,j]=np.mean([next(r for r in report['rows'] if r['size']==size and r['fps']==fps)['episode_capture_fraction'] for report in reports])
    ax=axes[0];im=ax.imshow(grid,vmin=0,vmax=1,cmap='YlGnBu',aspect='auto')
    ax.set_xticks(range(3),rates);ax.set_yticks(range(5),[f'{n} x {n}' for n in sizes]);ax.set_xlabel('Sampled FPS');ax.set_ylabel('Resolution')
    ax.set_title('Reference-readable windows retained\n2 clips, 3 sampling phases per condition')
    for i in range(5):
        for j in range(3):ax.text(j,i,f'{grid[i,j]*100:.0f}%',ha='center',va='center',color='white' if grid[i,j]>.65 else 'black')
    fig.colorbar(im,ax=ax,label='Fraction of 640/15-readable episodes')
    ax=axes[1]
    for j,fps in enumerate(rates):
        bandwidth=np.asarray([n*n*3*fps*8/1e6 for n in sizes])
        ax.plot(bandwidth,grid[:,j],'-o',label=f'{fps} FPS')
        for n,b,y in zip(sizes,bandwidth,grid[:,j]):ax.annotate(str(n),(b,y),xytext=(3,4),textcoords='offset points',fontsize=8)
    ax.axvline(60,color='gray',ls='--',lw=1,label='60 Mbps payload screen (not safety limit)')
    ax.set(xlabel='One raw RGB stream payload (Mbps)',ylabel='Reference-readable window fraction',ylim=(0,1.07),title='Detail / sampling / bandwidth trade-off')
    ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower right')
    fig.suptitle('Frozen Mixed v1.2 decoder: replay screening, not physical robot reliability',fontsize=12)
    output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();plot(args.root,args.output)
