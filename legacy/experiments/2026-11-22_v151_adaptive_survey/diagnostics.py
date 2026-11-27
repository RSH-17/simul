"""Extract only mission events which the completed native recording actually logged."""
from collections import Counter
import json


def rows(path):
    if not path.is_file():
        return None
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def extract(run,row,metrics,expected):
    status=metrics.get('last_policy_status') or {}
    elapsed=row.get('elapsed_sim_s',0.)
    decisions=rows(run/'native/center_wall.jsonl')
    manifest=run/'policy_manifest.json'
    candidate=manifest.is_file() and 'v151-survey-candidate-' in run.name
    if decisions is not None:
        decisions=[x for x in decisions if -.01<=x.get('elapsed',-1)<=elapsed+.1]
    adaptive=([x for x in decisions if x.get('survey_kind')] if candidate and decisions is not None else None)
    log=run/'launch.log'
    text=log.read_text(errors='replace') if log.is_file() else None
    nav=None
    if text is not None and '[bt_navigator]:' in text:
        lines=[l.partition('[bt_navigator]: ')[2] for l in text.splitlines() if '[bt_navigator]:' in l]
        nav=dict(started=sum(l.startswith('Begin navigating from current location') for l in lines),
                 succeeded=sum(l.startswith('Goal succeeded') for l in lines),
                 canceled=sum(l.startswith(('Goal canceled','Goal cancelled')) for l in lines),
                 failed=sum(l.startswith(('Goal failed','Goal aborted')) for l in lines))
        nav['terminal_not_matched']=nav['started']-sum(nav[k] for k in ('succeeded','canceled','failed'))
    hints=None
    if candidate and text is not None and '[mission]:' in text:
        hints=sum('[mission]: Accepted tiny edge glance ' in line for line in text.splitlines())
    weak=None if decisions is None else sum(bool(r.get('weak_wall')) for r in decisions)
    seen_near={int(k) for k in metrics.get('registered_and_within_s',{})}
    ids=set(row.get('registered_ids',[]))
    map_errors={int(k):v for k,v in (row.get('map_errors_saved_m') or {}).items()}
    present_distances={int(k):v for k,v in metrics.get('min_ground_truth_m',{}).items()}
    reasons=[]
    if expected-ids:
        reasons.append('some_expected_ids_missing')
    if ids-expected:
        reasons.append('unexpected_id_registered')
    if ids.intersection(expected)-seen_near:
        reasons.append('registered_but_no_1_5m_witness')
    if row.get('policy_sensor_proof_complete') is False:
        reasons.append('incomplete_original_sensor_proof')
    if expected-map_errors.keys():
        reasons.append('missing_saved_map_ids')
    if any(v>.25 for v in map_errors.values()):
        reasons.append('saved_map_error_above_0_25m')
    if row.get('bumper_events',0)>0:
        reasons.append('bumper_contact')
    flow=status.get('navigation_flow') or {}
    camera=status.get('camera_diagnostics') or {}
    return dict(policy_outcome=row.get('reason'),failed_criteria=reasons,
        missing_ids=sorted(expected-ids),unexpected_ids=sorted(ids-expected),
        missing_after_registration_near_ids=sorted((expected & ids)-seen_near),
        physically_within_1_5m_without_registration=sorted(i for i,d in present_distances.items() if d is not None and d<=1.5 and i not in ids),
        min_actual_distance_m=metrics.get('min_ground_truth_m'),
        registered_sim_s=metrics.get('registered_s'),registered_and_within_sim_s=metrics.get('registered_and_within_s'),
        adaptive_dispatch_counts=None if adaptive is None else {k:sum(x['survey_kind']==k for x in adaptive) for k in ('wall','interior','frontier')},
        adaptive_reason_counts=None if adaptive is None else dict(Counter(x.get('reason') for x in adaptive)),
        reopened_face_dispatches=None if (not candidate or decisions is None or not any('reopened_face' in x for x in decisions)) else sum(bool(x.get('reopened_face')) for x in decisions),
        weak_wall_dispatches=weak,tiny_edge_glance_accepted=hints,
        nav2_goal_log=nav,stall_cancellations=flow.get('stalls_cancelled'),
        latest_confirmed_surface_cells=status.get('confirmed_surface_cells'),
        latest_confirmed_face_anchors=status.get('confirmed_face_anchors'),
        confirmed_box_tracks=camera.get('confirmed_boxes'),
        camera_observed_box_face_bins=camera.get('observed_box_faces'),
        camera_coverage_updates=camera.get('coverage_updates'),
        verified_four_face_objects=None,
        instrumentation='Class counts reflect policy dispatch callbacks. Nav2 lines cover full launch without per-goal identity join. Last status exposes global face evidence, omits a per-object four-face ledger. Null indicates unavailable.')
