"""Extend the fixed loop-gate comparison to completed official zigzag records.

No native process or policy is changed. The mapper receives saved sensors and
odometry; independent evaluation remains a separate command after completion.
"""
import argparse
import json
from pathlib import Path

import run_official_slam_gate_study as shared


def completed_pair(state):
    jobs=[j for j in state['jobs'] if j['number'] in (4,5)]
    if (len(jobs)!=2 or {j['role'] for j in jobs}!={'baseline','candidate'}
            or any(j['status']!='done' or j.get('outcome',{}).get('infrastructure_failure') is not False
                   for j in jobs)):
        raise ValueError('Require the completed verified original pair')
    if any(j['scenario']['arena']['shape']!='zigzag' for j in jobs):
        raise ValueError('Require the original zigzag pair')
    if len({j['scenario']['sha256'] for j in jobs})!=1:
        raise ValueError('Require identical original development scenes')
    return jobs


def prepare(campaign,previous,output):
    prior=json.loads((previous/'state.json').read_text())
    if prior['status']!='completed' or any(j['status']!='completed' for j in prior['jobs']):
        raise ValueError('Previous component study must be finished')
    frozen=json.loads((previous/'configuration.json').read_text())
    shared.verify(frozen['source_sha256'])
    model=json.loads((Path(prior['jobs'][0]['output'])/'configuration.json').read_text())
    if model['variants']!=['baseline','strict_loop'] or model['ground_truth_input'] is not False:
        raise ValueError('Preserve the predeclared mapper comparison')
    originals=completed_pair(json.loads((campaign/'state.json').read_text()))
    recordings=[]
    for job in originals:
        episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        saved=list(episode.glob('*/map.data'))
        if len(saved)!=1 or not saved[0].with_suffix('.posegraph').exists():
            raise ValueError('Require one finalized sensor and graph pair')
        recordings.append((job,episode,saved[0]))
    output.mkdir(parents=True,exist_ok=False)
    driver=Path(__file__).resolve()
    files=[driver,Path(shared.__file__).resolve(),Path(model['binary']),
           Path(model['solver_prefix'])/'lib/libosp_solver_budget.so',
           driver.with_name('replay_karto_component.cpp')]
    jobs=[]
    for original,episode,data in recordings:
        directory=output/f'case_{original["number"]:02d}';directory.mkdir()
        config=dict(variants=['baseline','strict_loop'],data=str(data),posegraph=str(data.with_suffix('.posegraph')),
            binary=model['binary'],solver_prefix=model['solver_prefix'],driver=str(driver),
            original_mapper_parameters_copied=True,solver_threads=2,ground_truth_input=False)
        shared.save(directory/'configuration.json',config)
        files.extend([data,data.with_suffix('.posegraph'),directory/'configuration.json'])
        jobs.append(dict(number=original['number'],shape='zigzag',episode=str(episode),
            scenario_file=original['scenario']['file'],output=str(directory),status='prepared'))
    shared.save(output/'configuration.json',dict(source_sha256={str(p):shared.digest(p) for p in files},
        native_mission=False,ground_truth_input=False,maximum_simultaneous_components=1,
        hardware_validated=False,production_promoted=False,previous_fixed_study=str(previous)))
    shared.save(output/'state.json',dict(status='prepared',jobs=jobs))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--campaign',type=Path);p.add_argument('--previous',type=Path)
    p.add_argument('--run-prepared',action='store_true');a=p.parse_args()
    if a.run_prepared:shared.run(a.output.resolve())
    else:prepare(a.campaign.resolve(),a.previous.resolve(),a.output.resolve())
