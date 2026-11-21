"""Compare recorded goal alternatives with the measured route to one actual print."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from geometry import polygon, wall_segments
from surface_audit import blockers, readable_indices
from exposure_opportunity import pose


def read(path):
    return json.loads(path.read_text())


def analyze(run, ident, calibration):
    scene = read(run/'scene.json')
    target = next(t for t in scene['targets'] if t['id'] == ident)
    world = ET.parse(run/'world/osp_studio.sdf').find('.//world')
    model = world.find(f"model[@name='target_{ident}']")
    paper = pose(model.find("model[@name='paper']"))
    ink = pose(next(piece for piece in model.findall('model') if piece.get('name').startswith('cell_')))
    if ink[0] == paper[0]:
        raise ValueError('No unambiguous printed side')
    axis = np.array([math.cos(target['yaw']), math.sin(target['yaw']), 0.])
    normal = np.sign(ink[0]-paper[0])*axis
    tangent = np.array([-normal[1],normal[0],0.])
    center = np.array([target[k] for k in ('x','y','z')])+ink[0]*axis
    mount = np.array([target['x'],target['y']])
    camera_z = calibration['translation'][2]
    solids = blockers(world, min(camera_z,center[2]-target['size']/2), max(camera_z,center[2]+target['size']/2))
    audit = read(run/'audit.json')
    align = audit['world_from_map_initial_alignment']
    rot = np.array(align['rotation'])
    delta_yaw = math.atan2(rot[1,0],rot[0,0])
    camera_mount = Rotation.from_quat(calibration['quaternion'])
    log = run/'native/center_wall.jsonl'
    selections = [json.loads(s) for s in log.read_text().splitlines() if s.strip()] if log.exists() else []
    totals = Counter()
    matching = []
    shown = []
    for f in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(f,allow_pickle=False) as data:
            columns = list(data['candidate_columns'])
            options = data['candidates'][:, [columns.index(name) for name in ('x','y','yaw')]]
            t = float(data['elapsed'])
            chosen = int(data['selected_index'])
        totals['decisions'] += 1
        if not len(options):
            continue
        loc = options[:,:2]@rot.T + align['translation']
        near = np.flatnonzero(np.linalg.norm(loc-mount,axis=1) <= 1.5)
        if not len(near):
            continue
        yaw = delta_yaw + options[near,2]
        base = Rotation.from_euler('z',yaw[:,None])
        origins = base.apply(calibration['translation']) + np.column_stack((loc[near],np.zeros_like(yaw)))
        axes = (base*camera_mount).as_matrix()
        indices, _ = readable_indices(center, normal, tangent, target['size'], 18., origins, axes, calibration, solids)
        valid = near[indices]
        front = near[(origins-center)@normal > 0.]
        totals['options_close_front'] += len(front)
        totals['selected_close_front'] += bool(chosen in front)
        totals['options_readable_at_goal_heading'] += len(valid)
        totals['decisions_with_readable_option'] += bool(len(valid))
        totals['selected_readable_at_goal_heading'] += bool(chosen in valid)
        if len(valid) or chosen in front:
            entry = min(selections,key=lambda s:abs(s['elapsed']-t)) if selections else None
            same = entry is not None and abs(entry['elapsed']-t) < .15
            matching.append(dict(elapsed_s=t, proposed_xy=loc[valid].round(3).tolist(),
                                 selected_readable=bool(chosen in valid), selected_front=bool(chosen in front),
                                 selected_kind=entry['kind'] if same else None,
                                 selection_reason=entry['reason'] if same else None))
            for k in valid:
                shown.append((loc[k], bool(k==chosen)))
    if totals['decisions'] == 0:
        raise ValueError('No recorded policy choices')
    track = np.genfromtxt(run/'headings.csv', delimiter=',', names=True)
    track = track[(track['elapsed_sim_s']>=0)&(track['elapsed_sim_s']<=audit['elapsed_sim_s'])]
    t = track['elapsed_sim_s']
    if len(t) < 2 or np.any(np.diff(t)<0):
        raise ValueError('No ordered in-mission trajectory')
    xy = np.column_stack((track['x'],track['y']))
    dt = np.minimum(.5,np.diff(t,append=t[-1]))
    near = np.linalg.norm(xy-mount,axis=1)<=1.5
    front = (xy-center[:2])@normal[:2]>0.
    result = dict(run=audit['run'], seed=scene['seed'], id=ident, host=target['attached_to'],
                  printed_outward_xy=normal[:2].tolist(),
                  elapsed_sim_s=audit['elapsed_sim_s'], registered_s=audit['registered_s'].get(str(ident)),
                  body_near_front_s=round(float(dt[near&front].sum()),3),
                  body_near_back_s=round(float(dt[near&~front].sum()),3),
                  decisions=totals['decisions'], options_close_front=totals['options_close_front'],
                  selected_close_front=totals['selected_close_front'],
                  options_readable_at_goal_heading=totals['options_readable_at_goal_heading'],
                  decisions_with_readable_option=totals['decisions_with_readable_option'],
                  selected_readable_at_goal_heading=totals['selected_readable_at_goal_heading'], evidence=matching)
    return result, (xy,near,front,shown,scene,target,normal,mount,solids)


def figure(rows, panels, labels, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle,Polygon
    fig, axes = plt.subplots(1,2,figsize=(10,5.8),sharex=True,sharey=True)
    for row, data, name, ax in zip(rows,panels,labels,axes):
        xy,near,front,shown,scene,target,n,mount,solids = data
        for solid in solids:
            if solid.name.startswith('demo_furniture/'):
                corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*solid.half[:2]
                ax.add_patch(Polygon(corners@solid.axes[:2,:2].T+solid.center[:2],color='#a59983',zorder=2))
        for body in [*scene['partitions'],*scene['obstacles']]:
            ax.add_patch(Polygon(polygon(body),color='#9ba3ac',zorder=2))
        for name_wall, start, end in wall_segments(scene):
            if not name_wall.startswith('structure_'):
                ax.plot(*np.array([start,end]).T,color='#344554',lw=2)
        ax.add_patch(Circle(mount,1.5,fill=False,edgecolor='#838c92',linestyle='--'))
        ax.plot(xy[:,0],xy[:,1],color='#aab3bb',lw=1.2)
        for mask,color in ((near&front,'#158167'),(near&~front,'#ad5729')):
            path=np.where(mask[:,None],xy,np.nan)
            ax.plot(path[:,0],path[:,1],color=color,lw=2.7)
        if shown:
            for selected,mark,color in ((False,'o','#796497'),(True,'*','#2064a8')):
                points=np.array([point for point,chosen in shown if chosen==selected])
                if len(points):
                    ax.scatter(*points.T,marker=mark,s=88 if selected else 45,
                               facecolor=color if selected else 'none',edgecolor=color,zorder=4)
        tangent=np.array([-n[1],n[0]])*target['size']/2
        ink=np.array([mount-tangent,mount+tangent])
        ax.plot(*ink.T,color='black',lw=5,solid_capstyle='butt',zorder=5)
        ax.annotate('',xy=mount+.6*n[:2],xytext=mount+.05*n[:2],
                    arrowprops=dict(arrowstyle='->',color='black',lw=1.7))
        ax.set(xlim=(mount[0]-2.7,mount[0]+2.7),ylim=(mount[1]-2.6,mount[1]+1.6),aspect='equal',
               xlabel='World x (m)',title=f"{name}\n{row['body_near_front_s']:.1f} s in front within 1.5 m")
        ax.grid(alpha=.15)
    axes[0].set_ylabel('World y (m)')
    fig.suptitle(f"Random exam {rows[0]['seed']}  |  Printed face of marker {rows[0]['id']}")
    handles=[Line2D([],[],color='#158167',lw=3,label='Measured path in front'),
             Line2D([],[],color='#ad5729',lw=3,label='Measured path behind print'),
             Line2D([],[],marker='o',markerfacecolor='none',color='#796497',lw=0,label='Readable proposed heading'),
             Line2D([],[],marker='*',color='#2064a8',lw=0,markersize=10,label='Readable selected heading'),
             Line2D([],[],color='black',lw=3,label='Print and outward direction')]
    fig.legend(handles=handles,loc='lower center',ncol=3,frameon=False,fontsize=9)
    fig.subplots_adjust(left=.08,right=.98,bottom=.24,top=.85,wspace=.12)
    fig.savefig(output,dpi=155,bbox_inches='tight')
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recordings',type=Path,nargs=2)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--id',type=int,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plot',type=Path)
    p.add_argument('--labels',nargs=2,default=['Balanced','Candidate'])
    args=p.parse_args()
    calibration=read(args.calibration)
    results,panels=zip(*(analyze(run,args.id,calibration) for run in args.recordings))
    if results[0]['seed']!=results[1]['seed'] or any(
        hashlib.sha256((args.recordings[0]/part).read_bytes()).digest() !=
        hashlib.sha256((args.recordings[1]/part).read_bytes()).digest()
        for part in ['scene.json','world/osp_studio.sdf']):
        raise ValueError('The two recordings have different worlds')
    summary=dict(labels=args.labels, runs=results, minimum_px=18, max_incidence_deg=60,
                 scope='Arrival yaw only, initial map-to-world alignment. Saved alternatives include the base planning pool and the selected hybrid goal. Five SDF rays are geometric opportunities, not completed motion or successful Nav2 plans. In-place sweeps and subsequent turns are not counted.')
    args.output.write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    if args.plot:
        figure(results,panels,args.labels,args.plot)
