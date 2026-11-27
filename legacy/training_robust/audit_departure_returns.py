"""Evaluator-only corridor re-entry counts after complete body exit.

This measures a geometric return, not whether a later return was necessary for
the hidden-marker task. No resulting value is sent to the policy.
"""
import csv
import hashlib
import math
from pathlib import Path

import numpy as np
import yaml


def metrics(scene,rows):
    if not rows:raise ValueError('Need independent trajectory samples')
    yaw=scene['official_start']['exit_heading_rad'];start=np.asarray(scene['start'][:2])
    direction=np.array([math.cos(yaw),math.sin(yaw)]);side=np.array([-direction[1],direction[0]])
    points=np.asarray([[float(r['world_x']),float(r['world_y'])] for r in rows])-start
    along=points@direction;across=points@side
    length=scene['crowded_start']['exit_length_m'];width=scene['crowded_start']['corridor_width_m'];margin=.189+.01
    if width<=2*margin:raise ValueError('Corridor cannot contain the complete body')
    exited=False;first_exit=None;returns=[];origin=float(rows[0]['sim_seconds'])
    for row,p,q in zip(rows,along,across):
        t=float(row['sim_seconds'])-origin
        if p>=length+margin:
            exited=True
            if first_exit is None:first_exit=t
        elif exited and margin<=p<=length-margin and abs(q)<=width/2-margin:
            returns.append(t);exited=False
    return dict(first_full_exit_seconds=first_exit,full_body_corridor_reentries=len(returns),
        reentry_seconds=returns,final_exit_axis_progress_m=float(along[-1]),
        maximum_exit_axis_progress_m=float(along.max()),
        body_radius_plus_margin_m=margin,criterion='Entire body exits rail end, then entirely enters original rail corridor',
        interpretation='Geometric return count only; not a proof that a return was unnecessary')


def audit(episode,scenario):
    trace=episode/'ground_truth_trajectory.csv';scene=yaml.safe_load(scenario.read_text())
    with trace.open() as stream:result=metrics(scene,list(csv.DictReader(stream)))
    result.update(ground_truth_supplied_to_policy=False,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__).resolve(),trace,scenario]})
    return result
