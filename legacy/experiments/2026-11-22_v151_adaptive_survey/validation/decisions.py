"""Prespecified engineering screen and descriptive paired held-out inference."""
import math
from statistics import mean


def screen(current):
    problems=[]
    gain=False
    for split,seeds in (('regression',{112601,112602,112603,112604,112605}),('heldout',{112801,112802})):
        pairs=current.get(split,{}).get('pairs',[])
        if len(pairs)!=len(seeds) or {p['seed'] for p in pairs}!=seeds:
            problems.append(split+': incomplete paired results')
            continue
        for p in pairs:
            a,b=p['baseline'],p['candidate']
            if (a['sha256']!=b['sha256'] or a['expected_ids']!=b['expected_ids'] or
                    a['configured_start']!=b['configured_start'] or p['settled_start_distance_m']>.15):
                problems.append(split+': unpaired world, start or target ground truth')
            if b['bumps'] or b['unexpected_ids']:
                problems.append(f"{split} {p['seed']}: bumper or unexpected registered ID")
        for key in ('wall_reg','box_reg','wall_near','box_near','strict'):
            delta=sum(int(p['candidate'][key])-int(p['baseline'][key]) for p in pairs)
            if delta<0:problems.append(f'{split}: fewer {key} ({delta})')
            if key in ('wall_near','box_near','strict') and delta>0:gain=True
    if not current.get('complete'):problems.append('seven-pair report incomplete')
    if not gain:problems.append('no strict or independently verified approach gain')
    return dict(eligible=not problems,reasons=problems)


def scored(row):
    return row['elapsed_s'] if row['strict'] else 600.


def pairs_report(plan, completed):
    by_key={(s['pair'],s['role']):s for s in completed}
    pairs=[]
    for task in plan:
        a=by_key.get((task['pair'],'baseline'));b=by_key.get((task['pair'],'candidate'))
        if a is None or b is None:continue
        valid=(a['valid'] and b['valid'] and a['world_sha256']==b['world_sha256']
               and math.dist(a['settled_xy'],b['settled_xy'])<=.15)
        pairs.append(dict(pair=task['pair'],seed=task['seed'],topology=task['topology'],
            repeat=task['repeat'],order=task['order'],valid=valid,baseline=a,candidate=b,
            capped_candidate_minus_baseline_s=scored(b)-scored(a) if valid else None,
            jointly_successful_time_delta_s=(b['elapsed_s']-a['elapsed_s']
                                            if valid and a['strict'] and b['strict'] else None)))
    all_valid=len(pairs)==len(plan) and all(p['valid'] for p in pairs)
    valid=[p for p in pairs if p['valid']]
    first=[p for p in valid if p['repeat']==0]
    wins=sum(p['candidate']['strict'] and not p['baseline']['strict'] for p in first)
    losses=sum(p['baseline']['strict'] and not p['candidate']['strict'] for p in first)
    both=sum(p['candidate']['strict'] and p['baseline']['strict'] for p in first)
    neither=len(first)-wins-losses-both
    n=wins+losses
    exact=min(1.,2*sum(math.comb(n,i) for i in range(min(wins,losses)+1))/2**n) if n else 1.
    qualified=all_valid and all(p['candidate']['strict'] for p in pairs)
    if not all_valid or len(first)!=6:
        effect='undetermined_incomplete_or_invalid'
    elif wins>losses and exact<.05:
        effect='evidence_of_higher_strict_success_on_test_suite'
    elif losses>wins and exact<.05:
        effect='evidence_of_lower_strict_success_on_test_suite'
    else:effect='undetermined'
    # Conditional on the exchangeability assumption for six distinct sampled
    # layouts. A single layout per deliberately different family cannot estimate
    # family-specific failure probabilities.
    deltas=[p['jointly_successful_time_delta_s'] for p in first if p['jointly_successful_time_delta_s'] is not None]
    return dict(complete_valid=all_valid,task_qualification=('met_on_all_tested_runs' if qualified else
                    'not_met' if any(not p['candidate']['strict'] for p in valid) else 'undetermined'),
        primary_paired_exact=dict(distinct_geometries=len(first),candidate_only=wins,baseline_only=losses,
            both_success=both,both_fail=neither,two_sided_p=exact,interpretation=effect),
        first_run_joint_success_time_differences_s=deltas,
        first_run_mean_capped_difference_s=mean(p['capped_candidate_minus_baseline_s'] for p in first) if first else None,
        idealized_zero_failure_95_upper_bound=(1.-.05**(1/6) if len(first)==6 and all(p['candidate']['strict'] for p in first) else None),
        rare_failure_note='A bound for six independent identically distributed trials would require that modeling assumption. Repeats on the same geometry are not independent new layouts.',
        promotion='never_automatic',pairs=pairs)
