"""Retrospective upper bound on east-wall viewing opportunities in the cropped run."""
import argparse
import json
from pathlib import Path
import numpy as np


def survey(scene, poses, stamps, camera, size):
    width,height=scene['room_size']
    step=next(p for p in scene['partitions'] if p['name']=='east_wall_step')
    lo=-height/2+.075;hi=height/2-.075
    edges=np.linspace(lo,hi,int(np.ceil((hi-lo)/.1))+1)
    y=(edges[1:]+edges[:-1])/2
    x=np.where(abs(y-step['y'])<step['sy']/2,step['x']-step['sx']/2,width/2-.075)
    stamps=np.unique(stamps[(stamps>=max(0,poses[0,0]))&(stamps<=poses[-1,0])])
    poses=poses[np.unique(poses[:,0],return_index=True)[1]]
    yaw=np.interp(stamps,poses[:,0],np.unwrap(poses[:,3]))
    cx=np.interp(stamps,poses[:,0],poses[:,1]);cy=np.interp(stamps,poses[:,0],poses[:,2])
    bx,by,bz=camera['translation'];c,s=np.cos(yaw),np.sin(yaw)
    cx+=c*bx-s*by;cy+=s*bx+c*by
    dx=x-cx[:,None];dy=y-cy[:,None];depth=c[:,None]*dx+s[:,None]*dy
    # The recorded optical +Z points along base +X and image +X along base -Y.
    assert np.allclose(camera['quaternion'],[-.5,.5,-.5,.5])
    k=np.array(camera['k']).reshape(3,3)
    u=k[0,2]+k[0,0]*(s[:,None]*dx-c[:,None]*dy)/np.maximum(depth,.001)
    v=k[1,2]-k[1,1]*(.32-bz)/np.maximum(depth,.001)
    distance=np.hypot(dx,dy);incidence=dx/np.maximum(.001,distance)
    geometric=(depth>0)&(u>=0)&(u<camera['width'])&(v>=0)&(v<camera['height'])&(incidence>.60)
    eligible=geometric&(distance>.25)&(distance<.85*k[0,0]*size*incidence/18.)
    last=np.full(len(x),-np.inf);hits=np.zeros(len(x),int)
    for stamp,mask in zip(stamps,eligible):
        accepted=mask&(stamp-last>=.15-1e-8);hits[accepted]+=1;last[accepted]=stamp
    return x,y,hits,len(stamps)

def analyze(folder,framefile,calibration,destination):
    scene=json.loads((folder/'scene.json').read_text())
    poses=np.genfromtxt(folder/'headings.csv',delimiter=',',skip_header=1)
    times=np.genfromtxt(framefile,delimiter=',',skip_header=1)[:,1]
    camera=json.loads(calibration.read_text())
    result=dict(run=folder.name,wall='east facing west including its inset',
                ground_truth_targets=scene['targets'],scope='Captured RGB timestamps; ideal level camera, ignores occlusion, exposure and blur. Opportunities only.')
    for size in (.08,.20):
        xx,yy,hits,count=survey(scene,poses,times,camera,size)
        result[str(size)]=dict(eligible_samples=int(np.sum(hits>=3)),samples=len(hits),rgb_frames=count,
                               fraction=float(np.mean(hits>=3)))
    x,y,hits,_=survey(scene,poses,times,camera,.20)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Rectangle
    fig,(ax,detail)=plt.subplots(1,2,figsize=(10.5,5.5),width_ratios=[5,1],layout='constrained');w,h=scene['room_size']
    ax.add_patch(Rectangle((-w/2,-h/2),w,h,fill=False,edgecolor='#696969',lw=1.2))
    from matplotlib.transforms import Affine2D
    for table in json.loads((folder/'demo_layout.json').read_text())['tables']:
        item=Rectangle((table['x']-table['sx']/2,table['y']-table['sy']/2),table['sx'],table['sy'],color='#e7ddca')
        item.set_transform(Affine2D().rotate_around(table['x'],table['y'],table.get('yaw',0))+ax.transData)
        ax.add_patch(item)
    for b in scene['obstacles']+scene['partitions']:
        ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#b9bdbb'))
    ax.plot(poses[:,1],poses[:,2],color='#3978b2',lw=1.1,label='Recorded trajectory')
    dy=(y[1]-y[0])/2
    ax.add_collection(LineCollection([[(x0,y0-dy),(x0,y0+dy)] for x0,y0 in zip(x,y)],
        colors=np.where(hits>=3,'#208568','#c04e32'),linewidth=5))
    for t in scene['targets']:
        ax.scatter(t['x'],t['y'],c='#29302d',marker='s',s=21)
        ax.annotate(str(t['id']),(t['x'],t['y']),xytext=(4,3),textcoords='offset points')
    ax.plot([],[],lw=4,c='#208568',label='East wall: 3 eligible RGB exposures')
    ax.plot([],[],lw=4,c='#c04e32',label='East wall: fewer than 3')
    ax.set_aspect('equal');ax.set_xlim(-w/2-.5,w/2+.5);ax.set_ylim(-h/2-1,h/2+.4)
    ax.set_title('Mixed v1.5.1-balanced · run 112602 · 20 cm projection')
    for col,marker in enumerate((.08,.20)):
        _,sy,counts,_=survey(scene,poses,times,camera,marker)
        detail.add_collection(LineCollection([[(col,y0-dy),(col,y0+dy)] for y0 in sy],
            colors=np.where(counts>=3,'#208568','#c04e32'),linewidth=13))
        detail.text(col,-h/2-.35,f'{100*np.mean(counts>=3):.0f}%',ha='center')
    detail.set_xlim(-.55,1.55);detail.set_ylim(-h/2-1,h/2+.4)
    detail.set_title('East wall');detail.set_xticks([0,1],['8 cm','20 cm'])
    detail.set_yticks([]);detail.tick_params(length=0)
    for spine in detail.spines.values():spine.set_visible(False)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.02),ncol=2,fontsize=9,frameon=False)
    ax.set_axis_off()
    destination.mkdir(exist_ok=True)
    fig.savefig(destination/'east_wall_opportunities.png',dpi=160);plt.close(fig)
    (destination/'east_wall_opportunities.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['0.08'],result['0.2'])

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('run','frames','calibration','output'):p.add_argument(name,type=Path)
    a=p.parse_args();analyze(a.run,a.frames,a.calibration,a.output)
