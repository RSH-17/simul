"""Run repeated team trials (one at a time, or two at once with --parallel 2) and keep a manifest of every attempt.

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


def run_trial(index, planned, mode, seed, workspace, duration, domain, env):
    """One trial through team-sim.sh; returns its manifest entry."""
    command = ['bash', str(REPO/'team-sim.sh'), 'run', str(workspace)]
    command += [str(seed)] if seed is not None else []
    command += ['--duration', str(duration)]
    command += ['--domain', str(domain)] if domain else []
    print(f'[{index}/{planned}] {mode} {seed or "new random layout"}' + (f' (domain {domain})' if domain else ''), flush=True)
    began, started = time.monotonic(), datetime.datetime.now().isoformat(timespec='seconds')
    done = subprocess.run(command, cwd=REPO, text=True, capture_output=True, env=env)
    output = done.stdout+done.stderr
    run = re.search(r'^RUN (\S+)', output, re.M)
    drawn = re.search(r'Random exam seed (\d+)', output)
    attempt = dict(index=index, mode=mode, seed=seed if seed else int(drawn.group(1)) if drawn else None,
                   run=run.group(1) if run else None, exit_code=done.returncode,
                   completed='COMPLETED' in output, wall_s=round(time.monotonic()-began, 1))
    if domain:
        attempt.update(domain=domain, started=started, ended=datetime.datetime.now().isoformat(timespec='seconds'))
    if done.returncode:
        attempt['error_tail'] = output.strip().splitlines()[-3:]
    print(f'    <- [{index}] exit {done.returncode}, {attempt["wall_s"]} s, {attempt["run"]}', flush=True)
    return attempt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('workspace', type=Path)
    parser.add_argument('--seeds', type=int, nargs='*', default=[112601, 112602, 112603, 112604, 112605])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--random', type=int, default=0, help='Fresh random exam layouts after the repeats')
    parser.add_argument('--random-seeds', type=int, nargs='*', default=[],
                        help='Replay earlier random exam seeds (paired comparison) before any fresh ones')
    parser.add_argument('--duration', type=float, default=600.)
    parser.add_argument('--parallel', type=int, choices=(1, 2), default=1,
                        help='Trials at once. 2 uses ROS domains 174 and 175; runs record concurrency=2 '
                             'and summarize.py keeps them apart from one-at-a-time runs.')
    parser.add_argument('--name', default='campaign')
    a = parser.parse_args()
    workspace = a.workspace.expanduser().resolve()
    out = home()/'campaigns'
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    manifest_path = out/f'{a.name}-{workspace.name}-{stamp}.json'
    env = dict(os.environ, TEAM_SIM_PARALLEL=str(a.parallel)) if a.parallel > 1 else dict(os.environ)
    conditions = {k: v for k, v in env.items() if k.startswith('TEAM_SIM_')}
    trials = plan(a.seeds, a.repeats, a.random, random_seeds=a.random_seeds)
    manifest = dict(name=a.name, workspace=str(workspace), started=stamp, duration_s=a.duration,
                    conditions=conditions, planned=len(trials), parallel=a.parallel, attempts=[])
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')

    def record(attempt):
        manifest['attempts'].append(attempt)
        manifest['attempts'].sort(key=lambda entry: entry['index'])
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')

    if a.parallel == 1:
        for index, (mode, seed) in enumerate(trials, 1):
            record(run_trial(index, len(trials), mode, seed, workspace, a.duration, None, env))
    else:
        import queue
        import threading
        pending, results = queue.Queue(), queue.Queue()
        for item in enumerate(trials, 1):
            pending.put(item)

        def worker(domain):
            while True:
                try:
                    index, (mode, seed) = pending.get_nowait()
                except queue.Empty:
                    return
                try:
                    results.put(run_trial(index, len(trials), mode, seed, workspace, a.duration, domain, env))
                except Exception as exc:  # keep the attempt in the manifest
                    results.put(dict(index=index, mode=mode, seed=seed, run=None, exit_code=None,
                                     completed=False, domain=domain, error_tail=[repr(exc)]))

        workers = [threading.Thread(target=worker, args=(domain,), daemon=True) for domain in (174, 175)]
        for thread in workers:
            thread.start()
        for _ in trials:
            record(results.get())
        for thread in workers:
            thread.join()
    manifest['finished'] = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    print('MANIFEST', manifest_path, flush=True)


if __name__ == '__main__':
    main()
