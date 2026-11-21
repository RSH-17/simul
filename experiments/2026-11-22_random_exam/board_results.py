"""Present adjudicated exam results on existing completed 8780 cards."""
import argparse
import json
from pathlib import Path
import time


def read(path):
    return json.loads(path.read_text())


def update(path, changes):
    original=read(path)
    revised={**original,**changes}
    if revised!=original:
        temporary=path.with_suffix('.exam.new')
        temporary.write_text(json.dumps(revised,ensure_ascii=False,indent=2)+'\n')
        temporary.replace(path)


def apply(card, episode):
    evaluation=card/'simulation_evaluation.json'
    if not evaluation.is_file():
        return False
    score=episode['score']
    ids=sorted(int(i) for i,marker in score['records'].items() if marker['verified_s'] is not None)
    if not score['valid']:
        label='계측 확인 필요'
    elif score['strict']:
        label='엄격 완주'
    else:
        label='일부 미달'
    update(evaluation,dict(evaluation=f"{label} · 등록 {score['registered']}/{score['expected']} · 접근 {len(ids)}/{score['expected']}",
                           marker_evaluation=', '.join(map(str,ids)) or '근거리 확인 0개',
                           exam_score=score))
    latest=card/'mission_results/flow/latest.json'
    if latest.is_file():
        status=read(latest)['status']
        targets=[dict(item,approached=int(item['id']) in ids) for item in status.get('targets',[])]
        update(latest,dict(status=dict(status,targets=targets)))
    return True


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('exam',type=Path)
    cli.add_argument('cards',type=Path)
    cli.add_argument('--follow',action='store_true')
    args=cli.parse_args()
    seen=set()
    while True:
        file=args.exam/'episodes.json'
        records=read(file) if file.is_file() else []
        for episode in records:
            if episode['name'] not in seen and apply(args.cards/episode['name'],episode):
                seen.add(episode['name'])
        state=args.exam/'status.json'
        terminal=state.is_file() and read(state)['state'] in ('complete','invalid_pair','infrastructure_hold')
        if not args.follow or terminal:
            break
        time.sleep(10)


if __name__=='__main__':main()
