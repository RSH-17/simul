"""Run a policy on a scenario set and write a self-describing run directory.

    runs/<utc>_<backend>_<policy>_<set>/
        config.json     resolved layered config, git commit, scenario set
        plan.json       one line per planned episode (always written)
        episodes.jsonl  record + verdict per finished episode
        summary.json    aggregate verdicts
"""
import json
import subprocess
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

from osp.evaluation import evaluator
from osp.core.paths import REPO, runs_dir
from osp.policies.base import build as build_policy
from osp.core.registry import BACKENDS, load_builtins


def git_state():
    def git(*args):
        try:
            return subprocess.run(['git', *args], cwd=REPO, capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    return dict(commit=git('rev-parse', 'HEAD'), dirty=bool(git('status', '--porcelain')))


def make_backend(cfg):
    load_builtins()
    return BACKENDS.get(cfg['backend']['name'])(cfg['backend'].get('settings') or {})


def dotted(row, path):
    node = row
    for part in path.split('.'):
        node = node.get(part) if isinstance(node, dict) else None
    return node


def _episode(task):
    """One episode; module-level so that a process pool can run it."""
    cfg, scenario, repeat, log_path = task
    backend = make_backend(cfg)
    policy = build_policy(cfg)
    mission = scenario.mission(cfg['mission']['marker_count'], cfg['mission']['duration_s'])
    record, truth = backend.run_episode(scenario, policy, mission, log_path=log_path)
    record.metadata['repeat'] = repeat
    verdict = evaluator.evaluate(record, truth, cfg['criteria'], cfg['mission']['marker_count'])
    return dict(record=record.to_json(), verdict=verdict)


def run(cfg, set_name, scenarios, *, execute, repeats=1, workers=None, out=None, echo=print):
    backend = make_backend(cfg)
    policy = build_policy(cfg)
    wrong = sorted({s.kind for s in scenarios} - set(backend.accepts))
    if wrong:
        raise ValueError(f'{backend.name} cannot run {", ".join(wrong)} scenarios (set {set_name!r}); '
                         f'it accepts {", ".join(backend.accepts)}')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = Path(out) if out else runs_dir() / f'{stamp}_{backend.name}_{policy.name}_{set_name.replace("/", "-")}'
    out.mkdir(parents=True, exist_ok=False)
    workers = workers or (cfg.get('run') or {}).get('workers') or 1
    if backend.parallel_limit is not None:
        workers = min(workers, backend.parallel_limit)
    (out / 'config.json').write_text(json.dumps(dict(
        config=cfg, scenario_set=set_name, episodes=len(scenarios) * repeats, repeats=repeats,
        workers=workers, executed=execute, created_utc=stamp, git=git_state()), indent=2) + '\n')

    tasks = []
    with (out / 'plan.json').open('w') as plan:
        for repeat in range(repeats):
            for scenario in scenarios:
                mission = scenario.mission(cfg['mission']['marker_count'], cfg['mission']['duration_s'])
                line = backend.describe(scenario, policy, mission)
                plan.write(json.dumps(dict(scenario=scenario.id, repeat=repeat, action=line)) + '\n')
                log = out / 'logs' / f'{scenario.id}_r{repeat}.log' if backend.name != 'fast_sim' else None
                tasks.append((cfg, scenario, repeat, str(log) if log else None))
    if not execute:
        for t in tasks:
            echo('PLAN ' + backend.describe(t[1], policy, t[1].mission(cfg['mission']['marker_count'],
                                                                         cfg['mission']['duration_s'])))
        echo(f'Dry run: {len(tasks)} episodes planned in {out}. Add --execute to run them.')
        return out, None
    if any(t[3] for t in tasks):
        (out / 'logs').mkdir()

    rows = []
    pool_type = ThreadPoolExecutor if backend.parallel_limit is not None else ProcessPoolExecutor
    with pool_type(max_workers=workers) as pool, (out / 'episodes.jsonl').open('w') as stream:
        for row in pool.map(_episode, tasks):
            rows.append(row)
            stream.write(json.dumps(row) + '\n')
            stream.flush()
            echo(f"{row['record']['scenario_id']}: {row['verdict']['status']}")
    group = (cfg.get('run') or {}).get('group_by')
    summary = evaluator.summarize(rows, (lambda r: dotted(r, group)) if group else None)
    summary.update(backend=backend.name, policy=policy.name, scenario_set=set_name, group_by=group)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return out, summary


def import_team_sim(results_dir, out, match=None, echo=print):
    """Re-score completed team_sim trials with the evaluator into a run directory."""
    from osp.backends.gazebo import import_trial
    from osp.core.config import base as base_config
    base = base_config()
    rows, skipped = [], []
    for run_dir in sorted(Path(results_dir).iterdir()):
        if not run_dir.is_dir() or (match and match not in run_dir.name):
            continue
        imported = import_trial(run_dir)
        if imported is None:
            skipped.append(run_dir.name)
            continue
        scenario, record = imported
        verdict = evaluator.evaluate(record, scenario, base['criteria'], base['mission']['marker_count'])
        rows.append(dict(record=record.to_json(), verdict=verdict))
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    with (out / 'episodes.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row) + '\n')
    summary = evaluator.summarize(rows, lambda r: (r['record']['policy'], r['record']['metadata']['concurrency'],
                                                   r['record']['duration_s']))
    summary.update(source=str(results_dir), skipped_incomplete=skipped, criteria=base['criteria'],
                   note='Grouped by (workspace, concurrency, duration_s). Compare only within the same concurrency and duration.')
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    echo(f'Imported {len(rows)} trials ({len(skipped)} incomplete skipped) into {out}')
    return out, summary
