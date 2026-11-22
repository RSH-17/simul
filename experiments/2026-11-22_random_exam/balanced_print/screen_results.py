"""Score distant hints against earlier real mapped evidence; layouts are the unit of replication."""
import argparse
from collections import Counter
import json
from pathlib import Path


def score_run(audit):
    ids = set(audit['registered_s']) | set(audit['distant_hint_native_only'])
    details = []
    for ident in sorted(ids, key=int):
        prediction = audit['distant_hint_native_only'].get(ident, {})
        first = prediction.get('first_range_s')
        error = prediction.get('first_xy_error_m')
        old = [times[ident] for times in (audit['first_live_candidate_s'],
               audit['first_live_observation_s'], audit['registered_s']) if times.get(ident) is not None]
        earlier = min(old) if old else None
        lead = earlier - first if earlier is not None and first is not None else None
        wrong = first is not None and (error is None or error > .25)
        details.append(dict(id=int(ident), hint_s=first, first_existing_mapped_evidence_s=earlier,
                            lead_s=lead, first_xy_error_m=error, wrong_first=wrong,
                            novel_early=first is not None and not wrong and (lead is None or lead >= 5.),
                            outlier_estimates=prediction.get('wrong_over_025m', 0)))
    return dict(run=audit['run'], frames=audit['recorded_frames'],
                processed_with_tf=audit['processed_with_tf'], markers=details)


def summarize(episodes, audits, protocol):
    seeds = protocol['untuned_layouts']
    chosen = [r for r in episodes if r['seed'] in seeds and not r.get('repeat', False)]
    runs, by_seed = [], Counter()
    for episode in chosen:
        name = episode['name']
        if name not in audits:
            continue
        row = score_run(audits[name])
        row.update(seed=episode['seed'], role=episode['role'])
        runs.append(row)
        by_seed[row['seed']] += sum(x['novel_early'] for x in row['markers'])
    pairs = {(row['seed'], row['role']) for row in runs}
    complete = pairs == {(s, role) for s in seeds for role in ('baseline','candidate')} and len(runs)==len(pairs)
    good = sum(n > 0 for n in by_seed.values())
    wrong = sum(m['wrong_first'] for r in runs for m in r['markers'])
    return dict(complete=complete, completed_runs=len(runs), novel_layouts=good,
                wrong_first_hints=wrong, advance=complete and good>=2 and wrong==0, runs=runs)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('episodes', type=Path)
    ap.add_argument('audits', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    protocol = json.loads(Path(__file__).with_name('distant_hint_screen.json').read_text())
    all_runs = json.loads(args.episodes.read_text())
    audits = {r['name']: json.loads(p.read_text()) for r in all_runs
              if (p := args.audits / (r['name']+'.json')).is_file()}
    result = summarize(all_runs, audits, protocol)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
