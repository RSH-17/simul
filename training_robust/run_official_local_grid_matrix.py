"""Run the new local-grid matrix and audit localization after each final result."""
import argparse
from pathlib import Path

import run_official_local_grid_screen as local
from audit_pose_trace import audit as audit_pose


def run(output):
    original_collect=local.screen.collect
    def collect(job):
        original_collect(job)
        episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        try:
            report=audit_pose(episode)
            local.screen.base.save(Path(job['output'])/'pose_audit.json',report)
            job['localization_audit']=report['pose_errors']
        except (OSError,ValueError,KeyError,IndexError) as exc:
            # An unavailable audit is explicit; never turn it into a pass or
            # overwrite the original evaluator result or independent escape.
            job['localization_audit_error']=repr(exc)
    local.screen.collect=collect
    local.run(output)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.output.resolve())
