"""Summarize a campaign manifest (tools/team_sim/campaign.py) into a table.

  python3 tools/team_sim/summarize.py MANIFEST.json [MORE.json ...] [--markdown OUT.md]

Every attempt counts, including failures and interrupted runs (as failures).
ID registration and the independent 1.5 m approach after registration are
reported separately, and wall-mounted and box-mounted markers are split.
Success rates carry a Wilson 95% interval so small samples are not over-read.
Runs recorded under different conditions (workspace source, render backend,
camera noise) are refused together unless --allow-mixed is given.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k/n
    centre = (p+z*z/(2*n))/(1+z*z/n)
    half = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    return max(0., centre-half), min(1., centre+half)


def kind(target):
    host = str(target.get('attached_to', ''))
    return 'wall' if host.startswith('wall') else 'box'


def condition(run):
    protocol = json.loads((run/'protocol.json').read_text())
    source = hashlib.sha256(json.dumps(protocol.get('policy_source_sha256', {}), sort_keys=True).encode()).hexdigest()[:12]
    noise = protocol.get('camera_noise') or {}
    noise = 'off' if not any(noise.get(k, 0) for k in ('sigma', 'blur', 'drop')) else \
        f"sigma={noise['sigma']},blur={noise['blur']},drop={noise['drop']}"
    return dict(source=source, render=protocol.get('render_backend', 'software'), camera_noise=noise,
                duration_s=protocol.get('duration_sim_s'))


def attempt_row(attempt):
    row = dict(mode=attempt['mode'], seed=attempt['seed'], run=attempt['run'], completed=attempt['completed'])
    run = Path(attempt['run']) if attempt['run'] else None
    metrics = run/'metrics.json' if run else None
    if not metrics or not metrics.is_file():
        # A failed attempt still misses its markers; count them when the layout is known.
        counts = {'wall': [0, 0, 0], 'box': [0, 0, 0]}
        scene = run/'scene.json' if run else None
        if scene and scene.is_file():
            for t in json.loads(scene.read_text())['targets']:
                counts[kind(t)][0] += 1
        return dict(row, valid=False, strict=False, registered=0, approached=0, markers=3,
                    wall=counts['wall'], box=counts['box'], end_s=None, path_m=None, bumps=None,
                    reason=(attempt.get('error_tail') or ['no metrics'])[-1])
    m = json.loads(metrics.read_text())
    targets = json.loads((run/'scene.json').read_text())['targets']
    registered = {str(i) for i in m.get('final_ids', [])}
    approached = {str(i) for i in (m.get('registered_and_within_s') or {})} & registered
    counts = {'wall': [0, 0, 0], 'box': [0, 0, 0]}  # markers, registered, approached
    for t in targets:
        c = counts[kind(t)]
        c[0] += 1
        c[1] += str(t['id']) in registered
        c[2] += str(t['id']) in approached
    finish = max((m.get('registered_and_within_s') or {}).values(), default=None) if m.get('strict_success') else None
    return dict(row, valid=True, strict=bool(m.get('strict_success')), registered=len(registered & {str(t['id']) for t in targets}),
                approached=len(approached), markers=len(targets), wall=counts['wall'], box=counts['box'],
                end_s=finish, elapsed_s=m.get('elapsed_sim_s'), path_m=m.get('path_length_m'),
                bumps=m.get('bumper_events'), reason=m.get('reason'), condition=condition(run))


def rate(rows, key):
    k = sum(r[key] for r in rows)
    return k, len(rows), wilson(k, len(rows))


def fmt_rate(k, n, ci):
    return f'{k}/{n}' + (f' ({ci[0]:.0%}–{ci[1]:.0%})' if ci else '')


def summarize(rows):
    def group(selected):
        if not selected:
            return None
        wall = [sum(r['wall'][i] for r in selected) for i in range(3)]
        box = [sum(r['box'][i] for r in selected) for i in range(3)]
        finished = [r['end_s'] for r in selected if r['end_s'] is not None]
        return dict(trials=len(selected), strict=rate(selected, 'strict'),
                    all_three_registered=sum(r['registered'] == r['markers'] for r in selected),
                    wall_registered=wall[1], wall_approached=wall[2], wall_markers=wall[0],
                    box_registered=box[1], box_approached=box[2], box_markers=box[0],
                    mean_finish_s_of_successes=sum(finished)/len(finished) if finished else None,
                    bumps=sum(r['bumps'] or 0 for r in selected),
                    invalid=sum(not r['valid'] for r in selected))
    legacy = [r for r in rows if r['mode'] == 'legacy']
    per_seed = {seed: group([r for r in legacy if r['seed'] == seed]) for seed in sorted({r['seed'] for r in legacy})}
    return dict(development_maps=group(legacy), per_seed=per_seed,
                unseen_random=group([r for r in rows if r['mode'] == 'random']))


def markdown(summary, conditions, manifests):
    lines = ['# Campaign summary', '', 'Manifests: ' + ', '.join(f'`{Path(m).name}`' for m in manifests), '',
             'Conditions: ' + '; '.join(f'source `{c["source"]}`, render {c["render"]}, camera noise {c["camera_noise"]}, '
                                        f'{c["duration_s"]:.0f} s' for c in conditions), '',
             '| Set | Trials | Strict success (95% CI) | All 3 registered | Wall reg / approach | Box reg / approach | Mean finish (successes) | Bumps |',
             '|---|---:|---|---:|---|---|---:|---:|']

    def line(name, g):
        if g is None:
            return
        mean = f"{g['mean_finish_s_of_successes']:.0f} s" if g['mean_finish_s_of_successes'] else '–'
        lines.append(f"| {name} | {g['trials']} | {fmt_rate(*g['strict'])} | {g['all_three_registered']} | "
                     f"{g['wall_registered']}/{g['wall_markers']} · {g['wall_approached']}/{g['wall_markers']} | "
                     f"{g['box_registered']}/{g['box_markers']} · {g['box_approached']}/{g['box_markers']} | {mean} | {g['bumps']} |")
    line('Development maps (all)', summary['development_maps'])
    for seed, g in summary['per_seed'].items():
        line(f'  map {seed}', g)
    line('Unseen random layouts', summary['unseen_random'])
    lines += ['', 'Strict success = 3 correct IDs, each approached within 1.5 m after registration (independent ground truth), '
              'near-range camera proof, map error ≤ 0.25 m, zero bumps. Failed and interrupted attempts are included.']
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('manifests', type=Path, nargs='+')
    parser.add_argument('--markdown', type=Path)
    parser.add_argument('--allow-mixed', action='store_true')
    a = parser.parse_args()
    rows = []
    for path in a.manifests:
        rows += [attempt_row(attempt) for attempt in json.loads(path.read_text())['attempts']]
    conditions = [dict(t) for t in {tuple(sorted(r['condition'].items())) for r in rows if r.get('condition')}]
    if len(conditions) > 1 and not a.allow_mixed:
        raise SystemExit('Runs from different conditions; summarize them separately or pass --allow-mixed:\n'
                         + json.dumps(conditions, indent=2))
    summary = summarize(rows)
    text = markdown(summary, conditions, a.manifests)
    print(text)
    out = a.markdown or a.manifests[0].with_suffix('.summary.md')
    out.write_text(text)
    out.with_suffix('.json').write_text(json.dumps(dict(conditions=conditions, summary=summary, attempts=rows), indent=2)+'\n')


if __name__ == '__main__':
    main()
