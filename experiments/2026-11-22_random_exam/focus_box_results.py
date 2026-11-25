"""Compare the five newly driven focus-policy episodes to the two original controls."""
import argparse
from pathlib import Path
import shutil
import tempfile

from box_metrics import outer_walls, paired_camera_summary
from export import export_video, read, sha, write
from visual_survey_results import endpoint, matched


def audit_camera(panel, row, recordings, calibration):
    from fixture import save
    from route_repeat import analyze as repeated
    from sensor_audit import audit_episode, bag_messages, epochs
    name = row['name']
    bag = panel / 'sensor-recordings' / name
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('Camera evidence is incomplete for ' + name)
    run = recordings / name
    camera = panel / 'sensor-audit' / (name + '.json')
    camera.parent.mkdir(exist_ok=True)
    if not camera.is_file():
        segment, = epochs(bag_messages(bag))
        save(camera, audit_episode(run, segment, calibration))
    sensor = read(camera)
    path = panel / 'route-repeats' / (name + '.json')
    path.parent.mkdir(exist_ok=True)
    if sensor['valid'] and not path.exists():
        save(path, repeated(run, panel, calibration, sensor))


def scores(groups, roles):
    summary = {}
    for role in roles:
        rows = [g[role] for g in groups]
        totals = {key: sum(row[key] for row in rows) for key in (
            'registered', 'expected', 'near', 'wall', 'wall_near', 'objects', 'object_near', 'strict', 'bump')}
        totals.update(runs=len(rows), false_ids=sum(len(r['false_ids']) for r in rows),
                      mean_capped_s=sum(r['capped_s'] for r in rows) / len(rows),
                      mean_travel_m=sum(r['travel_m'] for r in rows) / len(rows))
        summary[role] = totals
    return summary


def plot_paths(worlds, rows, recordings, root, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from routes import draw
    fig, axes = plt.subplots(len(rows), 3, squeeze=False, figsize=(16, 3.1*len(rows)),
                             layout='constrained')
    for index, (world, row) in enumerate(zip(worlds, rows, strict=True)):
        for column, (role, name) in enumerate((('balanced','Balanced 1.5.1'), ('survey','Visual Survey 1.5.2'),
                                                ('focus','Focus Continuity 1.5.2.1'))):
            case = row[role]
            ax = axes[index, column]
            draw(ax, root/'fixtures'/f"map-{world['seed']}", case, recordings,
                 'baseline' if role=='balanced' else 'candidate')
            ax.set_title(name + '    ' + ax.get_title(), fontsize=9)
        axes[index, 0].set_ylabel(f"{world['seed']}\n" + ('diagnostic' if row['diagnostic'] else 'untuned'), size=8)
    fig.suptitle('8780 exam | actual routes on the same five box layouts | 600 s limit per drive', fontsize=14)
    fig.savefig(output, dpi=145, bbox_inches='tight')
    plt.close(fig)


def export(panel, reference, recordings, calibration, destination):
    if read(panel/'status.json')['state'] != 'complete' or read(reference/'status.json')['state'] != 'complete':
        raise ValueError('Both native panels must finish')
    reg = read(panel/'registration.json')
    protocol = read(panel/'protocol.json')
    if (sha(panel/'registration.json') != protocol['registration_sha256'] or
            sha(reference/'protocol.json') != reg['reference_protocol_sha256']):
        raise ValueError('Registration mismatch')
    reruns = {r['pair']:r for r in read(panel/'episodes.json') if r['role']=='candidate'}
    previous = {p['pair']:p for p in read(reference/'comparison.json')['pairs'] if not p['repeat'] and p['valid']}
    if len(previous) != len(reruns) or len(reruns) != len(protocol['worlds']):
        raise ValueError('Incomplete pairing')
    roles = ('balanced','survey','focus')
    maps, media, raw = [], [], []
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'bundle'
        stage.mkdir()
        for pair, world in enumerate(protocol['worlds'], 1):
            candidate = reruns[pair]
            baseline, survey = (previous[pair][k] for k in ('baseline','candidate'))
            if (candidate['source_sha256'] != reg['candidate']['source_manifest_sha256'] or
                    candidate['world_sha256'] != world['files']['world/osp_studio.sdf']):
                raise ValueError('Unregistered source or world')
            audit_camera(panel, candidate, recordings, calibration)
            diagnostic = world['seed'] in reg['diagnostic_seeds']
            row = dict(seed=world['seed'],diagnostic=diagnostic)
            for role, entry, root in zip(roles,(baseline,survey,candidate),(reference,reference,panel),strict=True):
                result = endpoint(root, entry)
                if result['sensor_valid']:
                    sensors = read(root/'sensor-audit'/(entry['name']+'.json'))
                    result['outer_walls'] = outer_walls(read(recordings/entry['name']/'scene.json'),
                                                       sensors['surface_coverage'])
                row[role] = result
            matched(row['balanced'],row['focus'])
            matched(row['survey'],row['focus'])
            maps.append(row)
            raw.append(dict(balanced=baseline, survey=survey, focus=candidate, diagnostic=diagnostic))
            media.append(export_video(candidate,panel.name,recordings,stage))
        diagnostic = [r for r in maps if r['diagnostic']]
        independent = [r for r in maps if not r['diagnostic']]
        if not independent or not diagnostic:
            raise ValueError('Separate the development case from the independent drives')
        camera = {control:paired_camera_summary([dict(baseline=r[control],candidate=r['focus']) for r in independent])
                  for control in ('balanced','survey')}
        result = dict(maps=maps, all_five=scores(maps,roles), diagnostic=scores(diagnostic,roles),
                      untuned=scores(independent,roles), untuned_camera=camera,
                      reference_protocol_sha256=reg['reference_protocol_sha256'],
                      new_drives=len(media),reused_drives=2*len(media))
        write(stage/'comparison.json',result)
        write(stage/'controls.json',[{k:item[k] for k in ('balanced','survey')} for item in raw])
        write(stage/'media.json',media)
        write(stage/'episodes.json',[r['focus'] for r in raw])
        for file in ('status.json','protocol.json','registration.json'):
            shutil.copy2(panel/file,stage/file)
        for dirname in ('sensor-audit','route-repeats'):
            shutil.copytree(panel/dirname,stage/dirname)
        plot_paths(protocol['worlds'],raw,recordings,panel,stage/'routes.png')
        stage.rename(destination)
    return result


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    for name in ('panel','reference','recordings','calibration','destination'):
        cli.add_argument(name,type=Path)
    args = cli.parse_args()
    result = export(args.panel,args.reference,args.recordings,read(args.calibration),args.destination)
    print(result['untuned'],flush=True)
