"""Attach final, independently verified near counts to Visual Survey's simulation cards."""
import argparse
import json
from pathlib import Path
import time

from board_results import apply


def follow(panel,cards):
    seen=set()
    final_refresh=None
    while True:
        ledger=panel/'episodes.json'
        rows=json.loads(ledger.read_text()) if ledger.exists() else []
        for row in rows:
            name=row['name']
            if name in seen:continue
            label='Mixed v1.5.2 Visual Survey'
            if row['purpose']=='distant_development_only':label+=' · 원거리 단서 진단'
            if apply(cards/name,row,label):seen.add(name)
        status=json.loads((panel/'status.json').read_text())['state']
        if status in ('complete','invalid_pair','infrastructure_hold'):
            if len(seen)==len(rows):break
            if final_refresh is None:final_refresh=time.monotonic()+80
            if time.monotonic()>final_refresh:raise RuntimeError('Unfinalized board cards: '+str(set(r['name'] for r in rows)-seen))
        time.sleep(6)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('panel',type=Path);parser.add_argument('cards',type=Path)
    args=parser.parse_args();follow(args.panel,args.cards)
