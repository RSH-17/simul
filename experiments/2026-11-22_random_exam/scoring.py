"""Independent randomized-exam scoring, supporting arbitrary IDs and rosters."""
import math


def score(row, metrics, scene, duration=600.):
    targets={t['id']:t for t in scene['targets']}
    expected=set(targets)
    if not expected or len(expected)!=len(scene['targets']):
        raise ValueError('The enrolled printed IDs must be distinct')
    if metrics.get('judge_stopped_policy_on_truth') is not False:
        raise ValueError('A passive observer is required')
    def stamps(data):
        result={}
        for name,t in data.items():
            ident=int(name)
            if not math.isfinite(t):raise ValueError('Nonfinite observation time')
            if 0<=t<=duration:result[ident]=t
        return result
    registered=stamps(metrics['registered_s'])
    near=stamps(row['after_registration_near_s'])
    proof=stamps(metrics['sensor_verified_at_s'])
    IDs=expected & set(near) & set(registered) & set(proof)
    records={}
    for ident,t in targets.items():
        valid=ident in IDs and near[ident]>=registered[ident]-1.e-8
        records[ident]=dict(host=t['attached_to'],mounted='wall' if t['attached_to'].startswith('wall_') else 'obstacle',
                            registered_s=registered.get(ident),after_registration_near_s=near.get(ident),
                            onboard_near_proof_s=proof.get(ident),verified_s=max(near[ident],proof[ident]) if valid else None)
    verified=[v['verified_s'] for v in records.values() if v['verified_s'] is not None]
    elapsed=metrics['elapsed_sim_s']
    false=sorted(set(row['false_registered_ids_after_start'])|set(row['unexpected_ids'])|
                 (set(metrics['final_ids'])-expected))
    errors={int(i):v for i,v in row['map_errors_m'].items()}
    accurate=set(errors)==expected and all(math.isfinite(e) and e<=.25 for e in errors.values())
    full_time=max(verified) if len(verified)==len(expected) else None
    strict=bool(row['valid'] and full_time is not None and accurate and row['bumps']==0 and not false and
                expected==set(metrics['final_ids']) and metrics['policy_sensor_proof_complete'])
    return dict(strict=strict,goal_complete_by_s=full_time,elapsed_s=elapsed,travel_m=row['distance_m'],
                stop_reason=metrics['reason'],expected=len(expected),registered=len(expected & registered.keys()),
                verified=len(verified),records=records,wall=sum(v['mounted']=='wall' for v in records.values()),
                wall_near=sum(v['mounted']=='wall' and v['verified_s'] is not None for v in records.values()),
                obstacle=sum(v['mounted']=='obstacle' for v in records.values()),
                obstacle_near=sum(v['mounted']=='obstacle' and v['verified_s'] is not None for v in records.values()),
                false_ids=false,bumps=row['bumps'],max_saved_error_m=max(errors.values(),default=None),valid=row['valid'])
