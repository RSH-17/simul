"""Run repeated team trials one after another and keep a manifest of every attempt.

Example (5 development maps x 3, then 5 unseen random layouts):
  python3 tools/team_sim/campaign.py ~/simul-ws/events-baseline \
      --seeds 112601 112602 112603 112604 112605 --repeats 3 --random 5 \
      --name baseline-repeat

Every attempt is listed, including failed or interrupted ones; nothing is
re-run to replace a failure. Environment options (TEAM_SIM_RENDER,
TEAM_SIM_CAMERA_NOISE, ...) are passed through to each run unchanged.
Summarize with tools/team_sim/summarize.py <manifest>.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from setup import home  # noqa: E402


def plan(seeds, repeats, random_count, interleave=True, random_seeds=()):
    """Order trials so repeats of one seed are spread through the campaign."""
    trials = []
    rounds = [[('legacy', s) for s in seeds] for _ in range(repeats)]
    for batch in rounds if interleave else [[t for r in rounds for t in r]]:
        trials.extend(batch)
    trials.extend(('random', s) for s in random_seeds)
    trials.extend(('random', None) for _ in range(random_count))
    return trials


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--seeds', type=int, nargs='*', default=[112601, 112602, 112603, 112604, 112605])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--random', type=int, default=0, help='Fresh random exam layouts after the repeats')
    parser.add_argument('--random-seeds', type=int, nargs='*', default=[],
                        help='Replay earlier random exam seeds (paired comparison) before any fresh ones')
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--name', default='campaign')
    a = parser.parse_args()
    workspace = a.workspace.expanduser().resolve()
    out = home()/'campaigns'
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    manifest_path = out/f'{a.name}-{workspace.name}-{stamp}.json'
    conditions = {k: v for k, v in os.environ.items() if k.startswith('TEAM_SIM_')}
    manifest = dict(name=a.name, workspace=str(workspace), started=stamp, duration_s=a.duration,
                    conditions=conditions, planned=len(plan(a.seeds, a.repeats, a.random, random_seeds=a.random_seeds)), attempts=[])
    for index, (mode, seed) in enumerate(plan(a.seeds, a.repeats, a.random, random_seeds=a.random_seeds), 1):
        command = ['bash', str(REPO/'team-sim.sh'), 'run', str(workspace)]
        command += [str(seed)] if seed is not None else []
        command += ['--duration', str(a.duration)]
        print(f'[{index}/{manifest["planned"]}] {mode} {seed or "new random layout"}', flush=True)
        began = time.monotonic()
        done = subprocess.run(command, cwd=REPO, text=True, capture_output=True)
        output = done.stdout+done.stderr
        run = re.search(r'^RUN (\S+)', output, re.M)
        drawn = re.search(r'Random exam seed (\d+)', output)
        attempt = dict(index=index, mode=mode, seed=seed if seed else int(drawn.group(1)) if drawn else None,
                       run=run.group(1) if run else None, exit_code=done.returncode,
                       completed='COMPLETED' in output, wall_s=round(time.monotonic()-began, 1))
        if done.returncode:
            attempt['error_tail'] = output.strip().splitlines()[-3:]
        manifest['attempts'].append(attempt)
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
        print(f'    -> exit {done.returncode}, {attempt["wall_s"]} s, {attempt["run"]}', flush=True)
    manifest['finished'] = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    print('MANIFEST', manifest_path, flush=True)


if __name__ == '__main__':
    main()
