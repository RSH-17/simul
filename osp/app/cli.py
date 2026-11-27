"""Single entry point:  python -m osp <command> ...

  list {policies,backends,scenarios}
  config   --backend B --policy P [--set k=v ...]          show the merged config
  run      --backend B --policy P --scenarios SET [--execute] [--limit N] [--repeats R] [--workers W]
  prepare  --policy P [--execute]                         build a Gazebo workspace via team_sim
  import-team-sim [--results DIR] [--match TEXT]          re-score finished team_sim trials

Gazebo actions only print their commands unless --execute is given. This tool
never drives the physical robot; use ./robot.sh for that.
"""
import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone

import yaml

from osp.app import runner
from osp.core import config
from osp.scenarios import catalog
from osp.core.paths import runs_dir, team_sim_home
from osp.core.registry import BACKENDS, load_builtins


def _list(args):
    load_builtins()
    if args.what == 'backends':
        for name in BACKENDS.names():
            print(f'{name:10} accepts {", ".join(BACKENDS.get(name).accepts)}')
    elif args.what == 'policies':
        for name in config.available('policies'):
            data = yaml.safe_load((config.CONFIGS / 'policies' / f'{name}.yaml').read_text())['policy']
            print(f'{name:22} {data["status"]:17} backends: {", ".join(sorted(data["implementations"]))}'
                  f'  - {data.get("summary", "")}')
    else:
        for name in catalog.names():
            spec = catalog.describe(name)
            size = len(spec['seeds']) if spec['kind'] == 'team_sim_seeds' else len(catalog.load(name))
            print(f'{name:26} {spec["kind"]:15} {size:4}  {spec.get("note", "")}')
    return 0


def _config(args):
    cfg = config.load(args.backend, args.policy, args.set)
    print(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), end='')
    return 0


def _run(args):
    cfg = config.load(args.backend, args.policy, args.set)
    scenarios = catalog.load(args.scenarios)
    if args.limit:
        scenarios = scenarios[:args.limit]
    # The surrogate has no side effects, so it always executes.
    execute = args.execute or cfg['backend']['name'] == 'fast_sim'
    out, summary = runner.run(cfg, args.scenarios, scenarios, execute=execute, repeats=args.repeats,
                              workers=args.workers, out=args.out)
    if summary is not None:
        brief = {k: summary[k] for k in ('episodes', 'success', 'failure', 'unverified', 'measured_success')}
        print(json.dumps(brief))
        print('Results:', out)
    return 0


def _prepare(args):
    load_builtins()
    from osp.backends.gazebo import prepare_command
    from osp.policies.base import build
    policy = build(config.load('gazebo', args.policy, args.set))
    command = prepare_command(policy)
    if not args.execute:
        print('PLAN', ' '.join(command))
        print('Dry run. Add --execute to build the workspace.')
        return 0
    return subprocess.run(command).returncode


def _import(args):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = args.out or runs_dir() / f'{stamp}_import_team-sim'
    _, summary = runner.import_team_sim(args.results, out, args.match)
    for group, block in summary.get('groups', {}).items():
        print(f'{group:40} strict {block["success"]:3}/{block["episodes"]:<3}')
    return 0


def parser():
    p = argparse.ArgumentParser(prog='python -m osp', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)

    s = sub.add_parser('list', help='list policies, backends or scenario sets')
    s.add_argument('what', choices=('policies', 'backends', 'scenarios'))
    s.set_defaults(func=_list)

    def common(s, scenarios=False):
        s.add_argument('--backend', required=True)
        s.add_argument('--policy', required=True)
        s.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='override a config value')
        if scenarios:
            s.add_argument('--scenarios', required=True, help='scenario set name (see `list scenarios`)')

    s = sub.add_parser('config', help='print the merged configuration')
    common(s)
    s.set_defaults(func=_config)

    s = sub.add_parser('run', help='run a policy on a scenario set')
    common(s, scenarios=True)
    s.add_argument('--execute', action='store_true', help='actually launch Gazebo (fast_sim always runs)')
    s.add_argument('--limit', type=int)
    s.add_argument('--repeats', type=int, default=1)
    s.add_argument('--workers', type=int)
    s.add_argument('--out')
    s.set_defaults(func=_run)

    s = sub.add_parser('prepare', help='build the Gazebo workspace of a policy')
    s.add_argument('--policy', required=True)
    s.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    s.add_argument('--execute', action='store_true')
    s.set_defaults(func=_prepare)

    s = sub.add_parser('import-team-sim', help='re-score completed team_sim trials')
    s.add_argument('--results', default=str(team_sim_home() / 'results'))
    s.add_argument('--match', help='only trial directories whose name contains this text')
    s.add_argument('--out')
    s.set_defaults(func=_import)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return args.func(args)
    except (config.ConfigError, KeyError, ValueError, FileNotFoundError, TypeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
