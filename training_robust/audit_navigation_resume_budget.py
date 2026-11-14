"""Freeze mission event evidence of target timeouts extended by inspections.

This audit does not replay a trajectory or estimate saved mission time. The
interval ends when the arrived-at face's observation turn is consumed, so a
conservative allowance is subtracted for the unlogged send/arrival boundaries.
"""
import argparse
import hashlib
import json
from pathlib import Path


def lines(data):
    end=data.rfind(b'\n')+1
    return [json.loads(line) for line in data[:end].splitlines() if line.strip()]


def analyze(faces,inspections,sensor_interruptions):
    intervals=[];selected=None
    for event in faces:
        if event['event']=='selected':selected=event
        elif event['event'] in ['cleared','unavailable']:selected=None
        elif event['event']=='consumed' and selected is not None:
            intervals.append((selected['stamp'],event['stamp']));selected=None
    pauses=[];requested=None
    for event in inspections:
        if event['event']=='requested':requested=event
        elif event['event']=='finished' and requested is not None:
            pauses.append((requested['stamp'],event['stamp'],event['resumed_navigation']))
            requested=None
    result=[]
    for start,end in intervals:
        included=[p for p in pauses if start<=p[0]<p[1]<=end]
        # Require all interruptions to have retained the exact goal, and no
        # unmeasured sensor pauses in the recording's cumulative counter.
        supported=sensor_interruptions==0 and all(p[2] for p in included)
        bracket=end-start
        subtraction=sum(b-a+1. for a,b,_ in included)+2.
        lower_bound=max(0.,bracket-subtraction) if supported else None
        result.append(dict(selected_stamp=start,consumed_stamp=end,bracket_s=bracket,
            completed_inspection_intervals=included,inspection_resume_count=len(included),
            navigation_active_lower_bound_s=lower_bound,
            exceeds_original_65_s_allowance=supported and lower_bound>65.,
            assumptions='At most 1s resume scheduling per inspection and 2s total selection/send/arrival scheduling; no sensor pauses'))
    return result


def freeze(episode,output):
    if output.exists():raise FileExistsError('Preserve earlier evidence')
    blobs={n:(episode/n).read_bytes() for n in ['face_directed_looks.jsonl','near_pass_inspections.jsonl','live_status.json']}
    status=json.loads(blobs['live_status.json'])['status']
    intervals=analyze(lines(blobs['face_directed_looks.jsonl']),lines(blobs['near_pass_inspections.jsonl']),status['sensor_interruptions'])
    result=dict(evidence_type='Partial native event timing audit, not a new navigation outcome',episode=str(episode),
        sensor_interruptions=status['sensor_interruptions'],intervals=intervals,
        observed_extended_targets=sum(r['exceeds_original_65_s_allowance'] for r in intervals),
        native_improvement_proven=False,ground_truth_supplied_to_policy=False,
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in blobs.items()},
        audit_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Timing bounds include explicit scheduling assumptions, not per-action instrumentation.',
            'An earlier cancellation changes subsequent observations; no saved-time or success-rate claim.',
            'Completed face intervals only; active targets and cleared/unavailable looks are excluded.'])
    output.mkdir(parents=True,exist_ok=False)
    for name,data in blobs.items():(output/name).write_bytes(data)
    (output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['episode','output']:p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args();print(json.dumps(freeze(args.episode.resolve(),args.output.resolve()),indent=2))
