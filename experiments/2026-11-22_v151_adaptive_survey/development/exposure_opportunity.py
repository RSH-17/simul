"""Paired 112601 camera opportunity audit. Ground truth is an evaluator input only.

python exposure_opportunity.py <recordings> <camera_geometry.json> --output exposure-112601.json --plot box32-cameras-112601.png
Needs scene.json, world/osp_studio.sdf, headings.csv, frames.csv, and metrics.json in each recording.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

RUNS={'balanced':'v151-boxes5-track-m112601-trial1','r2':'v151-survey-candidate-track-m112601-1122'}


def pose(node):
    result=np.array(list(map(float,(node.findtext('pose') or '0 0 0 0 0 0').split())))
    if result.shape!=(6,) or np.any(np.abs(result[3:5])>1e-6):
        raise ValueError('Static geometry parser expects planar translation and yaw')
    return result


@dataclass
class Blocker:
    name: str
    center: np.ndarray
    half: np.ndarray
    axes: np.ndarray


def blockers(world,zlo,zhi):
    out=[]
    for model in world.findall('model'):
        if model.get('name').startswith('target_'):
            continue
        if model.findall('model'):
            raise ValueError('Unexpected nested non-target world model')
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                size=visual.findtext('geometry/box/size')
                if size is None:
                    if visual.find('geometry/plane') is None:
                        raise ValueError('Unexpected non-box occluder')
                    continue
                pos=np.zeros(3); yaw=0.
                for node in (model,link,visual):
                    a=pose(node)
                    pos += [a[0]*math.cos(yaw)-a[1]*math.sin(yaw),
                            a[0]*math.sin(yaw)+a[1]*math.cos(yaw),a[2]]
                    yaw += a[5]
                half=np.array(list(map(float,size.split())))/2
                if pos[2]-half[2]>zhi or pos[2]+half[2]<zlo:
                    continue
                c,s=math.cos(yaw),math.sin(yaw)
                axes=np.array([[c,-s,0],[s,c,0],[0,0,1]])
                out.append(Blocker(model.get('name')+'/'+visual.get('name'),pos,half,axes))
    return out


def occluded(origin, point, blocks):
    """Slab intersection of the open sight line with each oriented solid visual box."""
    direction=point-origin
    blocked=np.zeros(len(origin),bool)
    for b in blocks:
        o=(origin-b.center)@b.axes
        ray=direction@b.axes
        parallel=np.abs(ray)<1e-12
        impossible=(parallel&(np.abs(o)>b.half)).any(axis=1)
        divisor=np.where(parallel,1.,ray)
        a=(-b.half-o)/divisor
        z=(b.half-o)/divisor
        lo=np.where(parallel,-np.inf,np.minimum(a,z)).max(axis=1)
        hi=np.where(parallel,np.inf,np.maximum(a,z)).min(axis=1)
        blocked|=(~impossible)&(hi>=np.maximum(0.,lo))&(lo<.9998)&(hi>1e-5)
    return blocked


def spans(mask,t):
    ids=np.flatnonzero(mask)
    return [[round(float(t[q[0]]),3),round(float(t[q[-1]]),3),len(q)]
            for q in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1) if len(q)]


def measure(folder, calibration, figure_id=32, within_mission=False):
    scene=json.loads((folder/'scene.json').read_text())
    metrics=json.loads((folder/'metrics.json').read_text())
    world=ET.parse(folder/'world/osp_studio.sdf').find('.//world')
    headings=np.genfromtxt(folder/'headings.csv',names=True,delimiter=',')
    frames=np.genfromtxt(folder/'frames.csv',names=True,delimiter=',')
    t=frames['elapsed_sim_s']
    if within_mission:
        t=t[(t>=0.) & (t<=min(600.,metrics['elapsed_sim_s']))]
    if len(t)<2:raise ValueError('Two in-window exposures are needed to estimate viewing time')
    dt=np.diff(t,append=t[-1]+np.median(np.diff(t))).clip(0,.3)
    x,y,a=[np.interp(t,headings['elapsed_sim_s'],v) for v in (headings['x'],headings['y'],np.unwrap(headings['yaw_rad']))]
    dx,dy,z=calibration['translation']
    origin=np.column_stack((x+np.cos(a)*dx-np.sin(a)*dy,y+np.sin(a)*dx+np.cos(a)*dy,np.full_like(x,z)))
    qx,qy,qz,qw=calibration['quaternion']
    optical_forward=np.float64([2*(qx*qz+qy*qw),2*(qy*qz-qx*qw),1-2*(qx*qx+qy*qy)])
    optical_down=np.float64([2*(qx*qy-qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz+qx*qw)])
    if abs(optical_forward[2])>1e-6 or not np.allclose(optical_down,[0,0,-1],atol=1e-6):
        raise ValueError('This audit expects a level optical frame')
    yaw=a+math.atan2(optical_forward[1],optical_forward[0])
    fx,cx,fy,cy=[calibration['k'][i] for i in (0,2,4,5)]
    moving=np.hypot(np.gradient(x,t),np.gradient(y,t))
    stopped=(moving<.03)&(np.abs(np.gradient(a,t))<.09)
    zlo=min([z]+[q['z']-q['size']/2 for q in scene['targets']])
    zhi=max([z]+[q['z']+q['size']/2 for q in scene['targets']])
    static=blockers(world,zlo,zhi)
    rows={}
    figure=None
    for tag in scene['targets']:
        ident=int(tag['id']); key=str(ident)
        marker=world.find(f"model[@name='target_{ident}']")
        paper=pose(marker.find("model[@name='paper']"))
        first_cell=next(m for m in marker.findall('model') if m.get('name','').startswith('cell_'))
        cell=pose(first_cell)
        sign=float(np.sign(cell[0]-paper[0]))
        if sign==0:raise ValueError('Ambiguous printed face')
        axis=np.float64([math.cos(tag['yaw']),math.sin(tag['yaw'])])
        normal=sign*axis
        center=np.float64([tag['x']+cell[0]*axis[0],tag['y']+cell[0]*axis[1],tag['z']])
        tangent=np.float64([-normal[1],normal[0],0])*tag['size']/2
        vertical=np.array([0.,0.,tag['size']/2])
        horizontal=origin[:,:2]-center[:2]
        incidence=(horizontal@normal)/np.linalg.norm(horizontal,axis=1)
        near=np.hypot(x-tag['x'],y-tag['y'])<=1.5
        bearing=np.degrees(np.arctan2(horizontal[:,1],horizontal[:,0]))
        sectors=np.array(['east','north','west','south'])[(np.floor((bearing+45)/90).astype(int))%4]
        limits=np.array([calibration['width']-5,calibration['height']-5])
        front=incidence>0
        valid=front.copy()
        uv=[]
        clear=[]
        for point in (center-tangent-vertical,center+tangent-vertical,center+tangent+vertical,center-tangent+vertical,center):
            d=point-origin
            depth=np.cos(yaw)*d[:,0]+np.sin(yaw)*d[:,1]
            valid&=depth>.1
            u=cx+fx*(np.sin(yaw)*d[:,0]-np.cos(yaw)*d[:,1])/np.maximum(depth,.1)
            v=cy-fy*d[:,2]/np.maximum(depth,.1)
            uv.append(np.column_stack([u,v]))
            clear.append(~occluded(origin,point,static))
        quad=np.array(uv[:4])
        valid &= np.all((quad>=4)&(quad<=limits),axis=(0,2))
        shortest=np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=-1).min(axis=0)
        count_clear=np.sum(clear,axis=0)
        aim=np.arctan2(center[1]-origin[:,1],center[0]-origin[:,0])-yaw
        off_axis=np.abs(np.degrees(np.arctan2(np.sin(aim),np.cos(aim))))
        center_in_image=(depth>.1)&np.all((uv[-1]>=4)&(uv[-1]<=limits),axis=1)
        full=valid&(shortest>=18)
        tiny=valid&(shortest>=6)&(shortest<18)
        five=full&(count_clear==5)
        named=metrics['registered_s'].get(key)
        sec=lambda mask: round(float(dt[mask].sum()),3)
        rows[key]=dict(printed_normal_xy=normal.tolist(),printed_side_sdf_x_sign=sign,
                physical_near_frames=int(near.sum()),physical_near_s=sec(near),physical_near_spans=spans(near,t),
                near_center_in_image_frames=int((near & center_in_image).sum()),
                near_min_off_axis_deg=float(off_axis[near].min()) if near.any() else None,
                near_median_off_axis_deg=float(np.median(off_axis[near])) if near.any() else None,
                near_quasistationary_s=sec(near&stopped),
                near_direction_s={direction:sec(near&(sectors==direction)) for direction in ('east','north','west','south')},
                near_incidence_s=dict(front_to_60_deg=sec(near&(incidence>=.5)),front_oblique=sec(near&(incidence>0)&(incidence<.5)),
                                      behind=sec(near&(incidence<=0))),
                front_in_fov_18px_frames=int(full.sum()),center_ray_clear_18px_frames=int((full&clear[-1]).sum()),
                five_rays_clear_18px_frames=int(five.sum()),five_rays_clear_18px_within_1p5m_frames=int((near&five).sum()),
                first_5ray_18px_s=round(float(t[five][0]),3) if five.any() else None,
                five_rays_clear_18px_frames_before_registration=int((five&(t<(named if named is not None else np.inf))).sum()),
                near_5ray_18px_spans=spans(near&five,t),
                small_6_to_lt18_px=dict(clear_all=int((tiny&(count_clear==5)).sum()),
                    partial_1_to_4_rays_clear=int((tiny&(count_clear>0)&(count_clear<5)).sum()),
                    all_five_blocked=int((tiny&(count_clear==0)).sum())),
                online_decoded_count=metrics['last_detector_status'].get('decoded',{}).get(key,0),
                registered_s=named,registered_then_actual_near_s=metrics['registered_and_within_s'].get(key))
        if ident==figure_id and not tag['attached_to'].startswith('wall_'):
            host=next(o for o in scene['obstacles'] if o['name']==tag['attached_to'])
            figure=dict(host=host,tag=tag,normal=normal,origin=origin,yaw=yaw,x=x,y=y,t=t,dt=dt,near=near,
                        five=five, behind=incidence<=0)
    inputs={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in
            ('scene.json','headings.csv','frames.csv','world/osp_studio.sdf','metrics.json')}
    return dict(run=folder.name,source_sha256=inputs,static_blockers_at_task_height=len(static),targets=rows),figure


def plot(results, figures, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle, Polygon
    fig,axes=plt.subplots(1,2,figsize=(11,6.7),sharex=True,sharey=True)
    fig.subplots_adjust(left=.08,right=.98,bottom=.20,top=.80,wspace=.12)
    blue='#24679a';green='#008268';orange='#c65c25'
    for ax, name, d in zip(axes,RUNS,figures):
        host=d['host']; tag=d['tag']; n=d['normal'];mx,my=tag['x'],tag['y']
        ax.add_patch(Circle((mx,my),1.5,fill=False,linestyle='--',edgecolor='#8d999f',linewidth=1.))
        xy=np.column_stack((d['x'],d['y'])); line=np.stack((xy[:-1],xy[1:]),axis=1)
        ax.add_collection(LineCollection(line,colors='#b9c3cc',linewidths=2,zorder=1))
        near=d['near'];ax.add_collection(LineCollection(line[near[:-1]&near[1:]],colors=blue,linewidths=3,zorder=2))
        corners=np.float64([[-1,-1],[1,-1],[1,1],[-1,1]])*np.array([host['sx'],host['sy']])*.5
        a=host['yaw'];c,s=math.cos(a),math.sin(a)
        corners=corners@np.array([[c,s],[-s,c]])+[host['x'],host['y']]
        ax.add_patch(Polygon(corners,closed=True,facecolor='#dad6cc',edgecolor='#686457',zorder=3))
        tangent=np.float64([-n[1],n[0]])*tag['size']/2
        pp=np.array([[mx,my]-tangent,[mx,my]+tangent])
        ax.plot(pp[:,0],pp[:,1],color='#242424',linewidth=5,zorder=4,solid_capstyle='butt')
        ax.annotate('',xy=(mx+.55*n[0],my+.55*n[1]),xytext=(mx+.03*n[0],my+.03*n[1]),
                    arrowprops=dict(arrowstyle='->',color='#242424',lw=1.6),zorder=4)
        sel=np.flatnonzero(near)[::5] # equal simulated time spacing, 1 s at 5 Hz
        for mask,color in ((d['five'],green),(d['behind'],orange),(~d['five']&~d['behind'],'#596c80')):
            ids=sel[mask[sel]]
            ax.quiver(d['origin'][ids,0],d['origin'][ids,1],np.cos(d['yaw'][ids])*.34,np.sin(d['yaw'][ids])*.34,
                      angles='xy',scale_units='xy',scale=1,color=color,width=.007,headwidth=4,zorder=5)
        ax.set_xlim(mx-2.35,mx+2.35);ax.set_ylim(my-2.10,my+2.10)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('World x (m)')
        summary=results[name]['targets']['32']
        ax.set_title(('Mixed v1.5.1-balanced' if name=='balanced' else 'Mixed Adaptive R2')+
                     '\n'+f"{summary['physical_near_s']:.1f} s within 1.5 m  ·  {summary['five_rays_clear_18px_within_1p5m_frames']} eligible frames",
                     fontsize=11)
    axes[0].set_ylabel('World y (m)')
    fig.suptitle('Marker 32 on map 112601: body path and camera heading',fontsize=14)
    handles=[Line2D([],[],color=blue,lw=3,label='Body within 1.5 m'),Line2D([],[],color=green,marker='>',lw=0,label='Eligible front view'),
             Line2D([],[],color=orange,marker='>',lw=0,label='Behind print'),Line2D([],[],color='#596c80',marker='>',lw=0,label='Other front-side view'),
             Line2D([],[],color='#242424',lw=3,label='Printed face / outward arrow')]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.005),ncol=3,frameon=False,fontsize=9)
    fig.savefig(destination,dpi=165);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('recordings',type=Path)
    p.add_argument('calibration',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plot',type=Path)
    args=p.parse_args()
    calib=json.loads(args.calibration.read_text())
    results={};figures=[]
    for role,name in RUNS.items():
        results[role],fig=measure(args.recordings/name,calib)
        figures.append(fig)
    out=dict(scope='Only development map 112601. Interpolate Gazebo planar body truth at actual recorded camera exposure '
             'timestamps. Compose recorded simulator camera-to-base transform; no body pitch/roll. Draw ideal black square '
             'using native camera K, printed-side SDF sign, scene size and actual black cell plane. Require all projected '
             'corners at least 4 native pixels inside, shortest projected side >=18 px. Ray test the four corners and center '
             'against static SDF visual boxes at task height. Five clear samples do not prove an unoccluded interior, '
             'sufficient contrast, raw input or decodability. Ignore static floor plane and marker cell geometry as occluders. '
             'Near means body <=1.5m from tag. Exposure duration weighted by next frame interval capped at 0.3 s, '
             'stationary means <0.03 m/s and |yaw rate|<0.09 rad/s. Geographical directions are camera position from tag. '
             'Online read/registration/physical approach metrics come from the saved run, separately.', runs=results)
    args.output.write_text(json.dumps(out,indent=2)+'\n')
    if args.plot:plot(results,figures,args.plot)
