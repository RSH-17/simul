"""Seal the three spread-box recordings against their registration and draw their measured routes."""
import argparse
from pathlib import Path
import shutil
import tempfile

from export import export_video
from fixture import digest, read, save
from spread_three_trial import spacing


def routes(root, recordings, rows, destination):
    from routes import draw, plt, Line2D, Patch

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 13.2))
    fig.subplots_adjust(left=.045,right=.955,bottom=.065,top=.926,hspace=.27)
    fig.suptitle('Mixed v1.5.2.1  |  Three spread-box exams', y=.981, size=18)
    fig.text(.5,.949,'3 boxes  ·  2 box markers + 1 outer-wall marker  ·  600 s per run',ha='center',size=10)
    for ax, row in zip(axes, rows):
        score=row['score']
        draw(ax,root/'fixtures'/f"map-{row['seed']}",row,recordings,'focus')
        ax.set_title(f"Run {row['pair']}    |    Boxes {score['obstacle_near']}/2     Wall {score['wall_near']}/1"
                     f"     |     {score['elapsed_s']:.0f} s    {score['travel_m']:.1f} m",fontsize=11)
        ax.set_xlabel(str(row['seed']),color='#62717c',fontsize=8)
    keys=[Patch(color=color,label=label) for color,label in (
        ('#147847','ID + verified approach'),('#bf7719','ID only'),('#a63232','ID missed'))]
    keys.append(Line2D([],[],color='#202e3e',marker='o',markerfacecolor='white',lw=0,label='Start'))
    fig.legend(handles=keys,loc='lower center',ncol=4,frameon=False,bbox_to_anchor=(.5,.018),fontsize=9)
    fig.savefig(destination,dpi=145,facecolor='white')
    plt.close(fig)


def seal(root, recordings, out):
    protocol,registration=read(root/'protocol.json'),read(root/'registration.json')
    finished=read(root/'status.json')
    rows=read(root/'episodes.json')
    fixtures=protocol['worlds']
    if (finished.get('state')!='complete' or finished['episodes']!=finished['limit'] or
            finished['limit']!=len(rows) or len(rows)!=3 or len(fixtures)!=3 or
            digest(root/'registration.json')!=protocol['registration_sha256']):
        raise ValueError('Three terminal native episodes and their original registration are required')
    seeds=[row['seed'] for row in rows]
    if seeds!=registration['seeds'] or len(set(seeds))!=3 or len({row['name'] for row in rows})!=3:
        raise ValueError('Unexpected native rows')
    out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():
        raise FileExistsError(out)
    with tempfile.TemporaryDirectory(dir=out.parent) as temporary:
        stage=Path(temporary)/'results'
        stage.mkdir()
        media=[]
        results=[]
        for i,(world,row) in enumerate(zip(fixtures,rows),1):
            run=recordings/row['name']
            scene=read(run/'scene.json')
            score=row['score']
            if (row['pair']!=i or row['role']!='focus' or row['source_sha256']!=registration['source']['source_manifest_sha256']
                or row['seed']!=world['seed'] or world['files']['world/osp_studio.sdf']!=row['world_sha256']
                or spacing(scene)!=world['spacing'] or (score['wall'],score['obstacle'],score['expected'])!=(1,2,3)
                or set(map(int,score['records']))!=set(world['ids'])
                or any(digest(run/name)!=value for name,value in {**row['evidence'],**world['files']}.items())):
                raise ValueError('Changed trial, policy, or evidence in '+row['name'])
            metrics=read(run/'metrics.json')
            video=export_video(row,root.name,recordings,stage)
            media.append(video)
            results.append(dict(run=i,seed=row['seed'],name=row['name'],world_sha256=row['world_sha256'],start=row['start'],
                spacing=world['spacing'],score=score,elapsed_wall_s=metrics['elapsed_wall_s'],
                min_marker_range_during_mission_m=metrics['min_ground_truth_m'],evidence=row['evidence'],video=video['asset']))
        save(stage/'results.json',dict(policy='Mixed v1.5.2.1 Focus Continuity',source_sha256=rows[0]['source_sha256'],
            registration_sha256=protocol['registration_sha256'],known_marker_count=3,max_sim_s=600,
            trials=results,summary=read(root/'comparison.json')))
        for filename in ('episodes.json','protocol.json','status.json','registration.json','comparison.json'):
            shutil.copy2(root/filename,stage/filename)
        save(stage/'media.json',media)
        routes(root,recordings,rows,stage/'routes.png')
        stage.rename(out)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    for arg in ('exam','recordings','destination'):cli.add_argument(arg,type=Path)
    args=cli.parse_args()
    seal(args.exam,args.recordings,args.destination)
