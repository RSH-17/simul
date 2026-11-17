"""Guard final graph changes without pretending retained coordinates are verified.

Large frame corrections need fresh evidence before automatic acceptance. No
truth is available here. Retaining an online coordinate is a conservative data
preservation decision, not proof it matches the final corrected map.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

from osp_mission.graph_landmarks import finalize as propose,read_graph

MAXIMUM_AUTOMATIC_SHIFT_M=.25


def finalize(observations,graph,targets):
    final,report=propose(observations,graph,targets)
    unresolved=[]
    for row in report['rows']:
        if row.get('applied') and row['shift_m']>MAXIMUM_AUTOMATIC_SHIFT_M+1e-9:
            row.update(applied=False,reason='large_graph_correction_requires_reobservation',
                proposed_point=row.pop('point'),retained_point=list(targets[row['id']]),coordinate_verified=False)
            final[row['id']]=tuple(targets[row['id']]);unresolved.append(row['id'])
        elif row.get('applied'):
            row['coordinate_verified']=True
        else:
            row['coordinate_verified']=False;unresolved.append(row['id'])
    report.update(corrected_ids=[r['id'] for r in report['rows'] if r['applied']],
        validation_version=1,maximum_automatic_shift_m=MAXIMUM_AUTOMATIC_SHIFT_M,
        requires_reobservation=unresolved,coordinates_verified=bool(targets) and not unresolved,
        verification_scope='Temporal multi-node consensus and bounded change, not independent absolute map accuracy')
    return final,report


def finalize_output(observations,targets,output,exporter):
    started=time.monotonic();output=Path(output);original={int(k):tuple(v) for k,v in targets.items()}
    report=dict(applied=False,ground_truth_inputs=False,corrected_ids=[],validation_version=1,
        coordinates_verified=False,requires_reobservation=list(original))
    try:
        graph=output/'graph_scan_poses.csv';data=output/'map.data'
        if not data.is_file():raise ValueError('Serialized SLAM dataset unavailable')
        (output/'graph_marker_observations.json').write_text(json.dumps(observations,allow_nan=False)+'\n')
        with (output/'graph_export.log').open('w') as stream:
            subprocess.run([str(exporter),str(data),str(graph)],stdout=stream,stderr=stream,check=True,timeout=5)
        final,report=finalize(observations,read_graph(graph),original)
        report['applied']=bool(report['corrected_ids'])
        report['input_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [data,graph,output/'graph_marker_observations.json']}
        if report['applied']:
            shutil.copy2(output/'markers.csv',output/'markers_before_graph.csv')
            temporary=output/'markers.graph.tmp'
            with temporary.open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['id','x','y'])
                for ident,(x,y) in sorted(final.items()):writer.writerow([ident,x,y])
            temporary.replace(output/'markers.csv')
    except Exception as exc:
        final=original;report.update(applied=False,coordinates_verified=False,corrected_ids=[],
            requires_reobservation=list(original),error=repr(exc),original_coordinates_preserved=True)
    report.update(finalization_wall_seconds=time.monotonic()-started,driving_time_and_approach_credits_modified=False)
    (output/'graph_landmark_update.json').write_text(json.dumps(report,allow_nan=False,indent=2)+'\n')
    return final,report
