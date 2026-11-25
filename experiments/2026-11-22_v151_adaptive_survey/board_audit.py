"""Refresh a completed comparison card when the board cached an obsolete trace offset."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
from urllib.request import urlopen


def reconcile(run,cards,board):
    if not (run/'audit.json').is_file() or not (run/'metrics.json').is_file():
        return 'not finished'
    card=cards/run.name
    trace=card/'simulation_trace.jsonl'
    if not trace.is_file() or not (card/'simulation_evaluation.json').is_file():
        return 'no finalized board metadata'
    data=trace.read_bytes()
    if not data:
        return 'empty trace'
    last=json.loads(data.splitlines()[-1])
    if last.get('state')!='FINISHED':
        return 'no final trace'
    card_id=hashlib.sha1(str(card.resolve()).encode()).hexdigest()[:10]
    url=board.rstrip('/')+'/api/run?id='+card_id

    def read():
        with urlopen(url,timeout=10) as response:
            return json.load(response)

    def matches(current):
        return (current.get('id')==card_id and current.get('phase')=='finished'
                and current.get('state')=='FINISHED' and abs(current['sim_time']-last['elapsed']) <= .11)

    if matches(read()):
        return 'current'
    # Run.refresh resets only when it observes a shorter trace. Preserve the
    # verified final inode, let this card reload, then restore its exact bytes.
    with tempfile.TemporaryDirectory(dir=cards) as work:
        saved=Path(work)/'complete'
        empty=Path(work)/'empty'
        os.link(trace,saved)
        empty.touch()
        try:
            os.replace(empty,trace)
            read()
        finally:
            os.replace(saved,trace)
    if not matches(read()) or trace.read_bytes()!=data:
        raise RuntimeError('Could not refresh the completed dashboard card: '+run.name)
    return 'cache refreshed'
