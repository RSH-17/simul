"""Reuse the frozen pocket-screen protocol for its isolated frontier-source fix."""
import argparse
import json
from pathlib import Path

import run_official_pocket_screen as protocol


UNIT = 'osp-official-frontier-evidence-screen-v1.service'
DELTA = {'osp_mission/start_waypoints.py', 'test/test_pocket_frontier_evidence.py'}


def prepare(output, workspace, prior_screen, frontier):
    # The established protocol preserves original manifests, weights, scene
    # hashes, all input gates, two native slots and per-service memory limits.
    previous_unit, previous_delta = protocol.UNIT, protocol.DELTA
    try:
        protocol.UNIT, protocol.DELTA = UNIT, DELTA
        protocol.prepare(output, workspace, prior_screen, frontier)
    finally:
        protocol.UNIT, protocol.DELTA = previous_unit, previous_delta
    config_path, state_path = output/'configuration.json', output/'state.json'
    config = json.loads(config_path.read_text())
    source = Path(__file__).resolve()
    config['driver_hashes'][str(source)] = protocol.screen.base.digest(source)
    config['comparison_scope'] = 'Same 90-second pocket trials; frontier evidence source only'
    state = json.loads(state_path.read_text())
    for job in state['jobs']:
        job['unit'] = f'osp-official-frontier-evidence-v1-{job["number"]:02d}.service'
    state['scope'] = config['comparison_scope']
    protocol.screen.base.save(config_path, config)
    protocol.screen.base.save(state_path, state)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    for name in ['workspace', 'prior-screen', 'frontier']:
        p.add_argument('--'+name, type=Path)
    p.add_argument('--run-prepared', action='store_true')
    a = p.parse_args()
    if a.run_prepared:
        protocol.run(a.output.resolve())
    else:
        prepare(a.output.resolve(), a.workspace.resolve(), a.prior_screen.resolve(), a.frontier.resolve())
