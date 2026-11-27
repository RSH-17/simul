"""Descriptive topology results; the six-layout primary test remains fixed."""
from collections import defaultdict
from statistics import mean


def summarize(result):
    rows=defaultdict(list)
    for pair in result['pairs']:rows[pair['topology']].append(pair)
    report={}
    for topology,pairs in rows.items():
        valid=[p for p in pairs if p['valid']]
        groups={}
        for role in ('baseline','candidate'):
            measured=[p[role] for p in valid]
            wall=sum(x['expected_wall'] for x in measured);box=sum(x['expected_box'] for x in measured)
            groups[role]=dict(measured=len(measured),strict=sum(x['strict'] for x in measured),
                wall_denominator=wall,box_denominator=box,
                wall_registered=sum(x['wall_registered'] for x in measured),wall_approached=sum(x['wall_near'] for x in measured),
                box_registered=sum(x['box_registered'] for x in measured),box_approached=sum(x['box_near'] for x in measured),
                unexpected_ids=sum(len(x['unexpected_ids']) for x in measured),bumps=sum(x['bumps'] for x in measured),
                mean_capped_s=mean(x['elapsed_s'] if x['strict'] else 600. for x in measured) if measured else None)
        report[topology]=dict(scheduled_observed=len(pairs),invalid_pairs=len(pairs)-len(valid),roles=groups,
            repeat_outcomes_consistent={role:len({p[role]['strict'] for p in valid})==1 if len(valid)>=2 else None
                                        for role in ('baseline','candidate')},
            capped_candidate_minus_baseline_s=[p['capped_candidate_minus_baseline_s'] for p in valid],
            paired_successful_time_delta_s=[p['jointly_successful_time_delta_s'] for p in valid
                                            if p['jointly_successful_time_delta_s'] is not None])
    return report
