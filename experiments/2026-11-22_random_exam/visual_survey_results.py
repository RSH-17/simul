"""Cross-check new drives against saved controls, and export each new camera recording once."""
import argparse
import math
from pathlib import Path
from statistics import mean
import tempfile

import numpy as np

from export import export_video, read, sha, write
from geometry import polygon


def endpoint(panel,row):
    name=row['name']
    metrics=row['score']
    if not metrics['valid']:raise ValueError('Invalid drive '+name)
    sensor=panel/'sensor-audit'/(name+'.json')
    if not sensor.is_file():sensor=panel/'route-repeats'/(name+'.coverage.json')
    repeat=panel/'route-repeats'/(name+'.json')
    evidence=read(repeat) if repeat.is_file() else None
    camera=read(sensor) if sensor.is_file() else None
    if camera and camera['run']!=name or evidence and evidence['run']!=name:
        raise ValueError('Evidence from a different drive')
    surface=camera['surface_coverage'] if camera and camera['valid'] else {}
    return dict(run=name,seed=row['seed'],source_sha256=row['source_sha256'],world_sha256=row['world_sha256'],
        start=row['start'],registered=metrics['registered'],near=metrics['verified'],expected=metrics['expected'],
        wall=metrics['wall'],wall_near=metrics['wall_near'],objects=metrics['obstacle'],
        object_near=metrics['obstacle_near'],strict=metrics['strict'],elapsed_s=metrics['elapsed_s'],
        capped_s=metrics['goal_complete_by_s'] if metrics['strict'] else 600.,travel_m=metrics['travel_m'],
        bump=metrics['bumps'],false_ids=metrics['false_ids'],markers=metrics['records'],
        sensor_valid=bool(camera and camera['valid']),sensor_issues=camera['reasons'] if camera else ['no_finalized_sensor_audit'],
        wall_fraction=surface.get('wall',{}).get('fraction'),object_fraction=surface.get('obstacle',{}).get('fraction'),
        boxes_four=surface.get('boxes_all_four'),boxes=surface.get('boxes'),nav=camera['navigate'] if camera else None,
        repeat={scope:{k:v for k,v in evidence[scope].items() if k!='intervals'} for scope in ('recent','entire_mission')} if evidence else None,
        selections=evidence['policy_decisions'] if evidence else None)


def matched(a,b):
    delta=abs(math.remainder(a['start'][2]-b['start'][2],math.tau))
    if (a['world_sha256']!=b['world_sha256'] or a['seed']!=b['seed'] or
            math.dist(a['start'][:2],b['start'][:2])>.15 or delta>math.radians(5) or set(a['markers'])!=set(b['markers'])):
        raise ValueError('Controls and test do not match')


def aggregate(groups):
    trials={role:[g[role] for g in groups] for role in ('balanced','print','survey')}
    result={}
    for role, rows in trials.items():
        out={k:sum(r[k] for r in rows) for k in ('registered','near','expected','wall','wall_near','objects','object_near','strict','bump')}
        out.update(n=len(rows),mean_capped_s=mean(r['capped_s'] for r in rows),mean_travel_m=mean(r['travel_m'] for r in rows),
                   false_ids=sum(len(r['false_ids']) for r in rows))
        for key in ('wall_fraction','object_fraction','repeated_no_new_view_m','repeated_m'):
            if key.endswith('_m'):
                vals=[r['repeat']['recent'][key] for r in rows if r['repeat']]
            else:
                vals=[r[key] for r in rows if r[key] is not None]
            out[key]=dict(mean=mean(vals) if vals else None,n=len(vals))
        result[role]=out
    return result


def plot(routes,recordings,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon
    colors={'balanced':'#526777','print':'#AA6619','survey':'#087E74'}
    n=len(routes);cols=2 if n>1 else 1
    fig,axes=plt.subplots(math.ceil(n/cols),cols,figsize=(8*cols,4.9*math.ceil(n/cols)),squeeze=False)
    for ax,(title,g) in zip(axes.flat,routes):
        ref=recordings/g['balanced']['run']
        scene=read(ref/'scene.json');layout=read(ref/'run/simulation/demo_layout.json')
        for body in [*layout['tables'],*scene['partitions'],*scene['obstacles']]:
            ax.add_patch(Polygon(polygon(body),facecolor='#b6b0a5' if body in scene['obstacles'] else '#e4e0d8',edgecolor='#8a857c',linewidth=.5))
        w,h=np.array(scene['room_size'])/2
        recess=scene['south_recess']
        a,b=recess['x_min'],recess['x_max']; y=-h-recess['depth']
        coords=[(-w,-h),(a,-h),(a,y),(b,y),(b,-h),(w,-h),(w,h),(-w,h),(-w,-h)]
        ax.plot(*zip(*coords),color='#544e44',lw=1.)
        for role in colors:
            if role not in g:continue
            p=np.loadtxt(recordings/g[role]['run']/'headings.csv',delimiter=',',skiprows=1,ndmin=2)
            p=p[(p[:,0]>=0)&(p[:,0]<=g[role]['elapsed_s'])]
            ax.plot(p[:,1],p[:,2],c=colors[role],lw=1.4,alpha=.86,zorder=2)
        x,y,yaw=g['balanced']['start']
        ax.scatter(x,y,marker='o',facecolor='white',edgecolor='black',s=33,zorder=3)
        ax.arrow(x,y,.55*math.cos(yaw),.55*math.sin(yaw),width=.024,color='#242629',zorder=4)
        for mark in scene['targets']:
            x,y=mark['x'],mark['y']
            ax.scatter(x,y,marker='s',s=30,c='#ad2364',zorder=4)
            ax.annotate(str(mark['id']),(x,y),xytext=(3,4),textcoords='offset points',size=8)
        ax.set(title=title,aspect='equal',xlabel='x (m)',ylabel='y (m)')
    for ax in list(axes.flat)[n:]:ax.set_visible(False)
    fig.legend([Line2D([0],[0],lw=2,c=c) for r,c in colors.items() if r in routes[0][1]],
               [t for r,t in [('balanced','Balanced'),('print','Balanced Print'),('survey','Visual Survey')] if r in routes[0][1]],
               loc='upper center',ncol=3,frameon=False)
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(output,dpi=140,bbox_inches='tight');plt.close(fig)


def compare(panel,official,alternate,cluster,recordings,stage):
    state=read(panel/'status.json');reg=read(panel/'registration.json')
    if state['state']!='complete' or read(cluster/'status.json')['state']!='complete':
        raise ValueError('Both native comparisons must be finalized')
    if sha(official/'protocol.json')!=reg['original_protocol'] or sha(alternate/'protocol.json')!=reg['alternate_protocol']:
        raise ValueError('Reference enrollment changed')
    if sha(cluster/'registration.json')!=read(cluster/'protocol.json')['registration_sha256']:
        raise ValueError('Registered overlap diagnostic changed')
    evidence=read(official/'distant-screen/holdout.json')
    if not evidence['complete'] or state['screen_passed']!=evidence['advance']:raise ValueError('Unfinished feature screen')
    rows=read(panel/'episodes.json')
    if len(rows)!=(6 if evidence['advance'] else 7):raise ValueError('Missing new drive')
    source=reg['source']['source_manifest_sha256']
    groups=[]; extra=[]; media=[]
    controls={p:{r['name']:r for r in read(p/'episodes.json')} for p in (official,alternate)}
    for r in rows:
        if r['source_sha256']!=source:raise ValueError('Unexpected policy')
        c=endpoint(panel,r)
        parent=alternate if r['purpose']=='alternate_pose' else official
        b=endpoint(parent,controls[parent][r['controls']['baseline']]);p=endpoint(parent,controls[parent][r['controls']['candidate']])
        matched(b,c);matched(p,c)
        group=dict(purpose=r['purpose'],seed=r['seed'],balanced=b,print=p,survey=c,far_hints=r['far_hints'])
        if r['purpose']=='distant_development_only':
            if evidence['advance'] or not r['far_hints']:raise ValueError('Wrong far-hint diagnostic')
            extra.append(group)
        else:
            if r['far_hints']!=evidence['advance']:raise ValueError('Different primary policy options')
            groups.append(group)
        media.append(export_video(r,panel.name,recordings,stage))
    five=sorted([g for g in groups if g['purpose'] in ('development','untuned')],key=lambda g:next(
        i for i,w in enumerate(read(official/'protocol.json')['worlds']) if w['seed']==g['seed']))
    if len({r['seed'] for r in five})!=5 or len([r for r in five if r['purpose']=='untuned'])!=3:
        raise ValueError('Unexpected independent layouts')
    cluster_rows=read(cluster/'episodes.json')
    if len(cluster_rows)!=2:raise ValueError('Expected paired challenge')
    stress={('balanced' if r['role']=='baseline' else 'survey'):endpoint(cluster,r) for r in cluster_rows}
    matched(stress['balanced'],stress['survey'])
    if stress['survey']['source_sha256']!=source:raise ValueError('Different overlap policy')
    for r in cluster_rows:media.append(export_video(r,cluster.name,recordings,stage))
    summary=dict(five_independent=aggregate(five),three_untuned=aggregate([g for g in five if g['purpose']=='untuned']),
        maps=five,alternate=next(g for g in groups if g['purpose']=='alternate_pose'),development_only=extra,
        overlap=dict(setup=read(cluster/'start-comparison.json'),**stress),distant_screen=evidence,
        enrollment=sha(panel/'registration.json'),overlap_enrollment=sha(cluster/'registration.json'))
    write(stage/'comparison.json',summary);write(stage/'media.json',media)
    write(stage/'episodes.json',rows+cluster_rows)
    plot([(f"Seed {g['seed']}  ({g['purpose']})",g) for g in five]+[('Same 9 objects, alternate start',summary['alternate'])],recordings,stage/'routes.png')
    plot([('Start facing two partially occluding objects',stress)],recordings,stage/'overlap-routes.png')
    return summary


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    for key in ('panel','official','alternate','overlap','recordings','destination'):
        cli.add_argument(key,type=Path)
    args=cli.parse_args()
    if args.destination.exists():raise FileExistsError(args.destination)
    with tempfile.TemporaryDirectory(dir=args.destination.parent) as tmp:
        dest=Path(tmp)/'bundle';dest.mkdir()
        compare(args.panel,args.official,args.alternate,args.overlap,args.recordings,dest)
        dest.rename(args.destination)
