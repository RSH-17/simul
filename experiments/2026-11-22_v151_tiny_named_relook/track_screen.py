"""Predetermined decision rules for the original 8780 demo track only."""
import math
import re

ORDER = (112601, 112605, 112602, 112603, 112604)
GAIN = ('wall_near', 'box_near', 'strict')
NOREGRESS = ('wall_reg', 'box_reg', *GAIN)
GLANCE = re.compile(r'\bAccepted (tiny edge|subthreshold print (\d+)) glance (-?[\d.]+) deg\b')


def accepted_glances(lines):
    named, anonymous = [], 0
    for line in lines:
        found = GLANCE.search(line)
        if found:
            if found[2] is None:
                anonymous += 1
            else:
                named.append({'id': int(found[2]), 'turn_deg': float(found[3])})
    return dict(accepted_named=named, accepted_anonymous_count=anonymous)


def judge(pairs, final):
    seeds = ORDER if final else ORDER[:2]
    if len(pairs) != len(seeds) or {p['seed'] for p in pairs} != set(seeds):
        return dict(state='hold', reasons=['Incomplete or duplicate original-track pairs'])
    for p in pairs:
        a, b = p['baseline'], p['candidate']
        if (not a['valid'] or not b['valid'] or a['world_sha256'] != b['world_sha256']
                or a['configured_start']!=b['configured_start'] or a['wall_ids']!=b['wall_ids']
                or a['box_ids']!=b['box_ids'] or math.dist(a['settled_xy'], b['settled_xy']) > .15):
            return dict(state='hold',reasons=[f"{p['seed']}: invalid or mismatched independent evidence"])
    bad=[p['seed'] for p in pairs if p['candidate']['bumps'] or p['candidate']['unexpected_ids']]
    reasons=['bumper or false registered ID on '+str(bad)] if bad else []
    if final:
        delta={k:sum(int(p['candidate'][k])-int(p['baseline'][k]) for p in pairs) for k in NOREGRESS}
        reasons.extend(k+' regressed' for k,v in delta.items() if v<0)
        if not any(delta[k]>0 for k in GAIN):
            reasons.append('no strict or independently verified approach gain')
    else:
        from rescreen import early_screen
        old=[dict(seed=p['seed'],candidate={k:p['candidate'][k] for k in ('strict','bumps','unexpected_ids')}) for p in pairs]
        if (not early_screen(old)['eligible_to_continue'] or not any(
                p['candidate']['strict'] and not p['baseline']['strict'] for p in pairs)) and not reasons:
            reasons.append('neither of the two failure maps reached strict success')
        delta={}
    return dict(state='declined' if reasons else 'passed',reasons=reasons,delta=delta)


def predecessor(queue_state, r2_finished, r3_futility, r3_done):
    if queue_state == 'needs_investigation':
        return 'infrastructure_hold'
    if not r2_finished:
        return 'waiting_for_preceding_campaign'
    if (queue_state == 'skipped_r2_eligible' or
            (queue_state == 'futility_stop' and r3_futility) or
            (queue_state == 'complete' and r3_done)):
        return 'ready'
    return 'waiting_for_preceding_campaign'


def common_maps(r3, r4):
    comparisons=[]
    for seed in ORDER:
        if seed not in r3 or seed not in r4:
            continue
        a,b=r3[seed],r4[seed]
        valid=(a['valid'] and b['valid'] and a['world_sha256']==b['world_sha256']
               and a['configured_start']==b['configured_start'] and math.dist(a['settled_xy'],b['settled_xy'])<=.15)
        comparisons.append(dict(seed=seed,matched=valid,R3=a,R4=b,
            gained_near_ids=sorted(set(b['near_ids'])-set(a['near_ids'])) if valid else None,
            lost_near_ids=sorted(set(a['near_ids'])-set(b['near_ids'])) if valid else None,
            capped_time_difference_s=((b['elapsed_s'] if b['strict'] else 600)-
                                      (a['elapsed_s'] if a['strict'] else 600)) if valid else None))
    return comparisons
