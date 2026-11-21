"""Write a human-readable paired result directly from the adjudicated ledger."""
import argparse
import json
from pathlib import Path
from statistics import mean


def read(path):
    return json.loads(path.read_text())


def marker_ids(row):
    return '·'.join(str(i) for i in sorted(int(i) for i,v in row['score']['records'].items()
                                          if v['verified_s'] is not None)) or '없음'


def report(root):
    data=read(root/'comparison.json');protocol=read(root/'protocol.json')
    worlds={w['seed']:w for w in protocol['worlds']}
    lines=['# 8780 무작위 시험장 주행 결과','',
           f"등록한 {data['planned_pairs']}쌍 중 {data['paired']}쌍의 양쪽 주행이 끝났다. "
           f"같은 출발 자세와 월드, 온전한 계측을 확인한 쌍은 {data['valid_pairs']}개다.",'',
           '| 처음 주행한 배치 | 엄격 완주 | ID 등록 | 1.5m 접근 | 벽 접근 | 장애물 접근 | 범퍼 | 오등록 | 평균 소요(초) | 평균 거리(m) |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for key,title in (('baseline','Balanced'),('candidate','R5')):
        r=data['first_runs'][key]
        if r['runs']==0:continue
        lines.append(f"| {title} | {r['strict']}/{r['runs']} | {r['registered']}/{r['all_markers']} | "
                     f"{r['verified']}/{r['all_markers']} | {r['wall_verified']}/{r['walls']} | "
                     f"{r['obstacle_verified']}/{r['obstacles']} | {r['bumps']} | {r['false_registered_instances']} | "
                     f"{r['mean_time_with_failures_at_limit_s']:.1f} | {r['mean_travel_m']:.1f} |")
    lines+=['','시간은 엄격 완주의 마지막 근거리 증거 시각이며 미완주에는 600초를 배정했다. 아래 주행 시간은 실제 시뮬레이션 종료 시각이다.','',
            '| 배치 | 시드 | 책상 | 이동식 장애물 수 | 실제 등록 후 접근 ID (Balanced / R5) | 종료 시간(초) (Balanced / R5) | 엄격 완주 (Balanced / R5) | 비교쌍 |',
            '|---|---:|---|---:|---|---|---|---|']
    for p in data['pairs']:
        b,c=p['baseline'],p['candidate'];w=worlds[p['seed']]
        initial='반복' if p['repeat'] else '첫 주행'
        reason='유효' if p['valid'] else '계측 또는 출발 자세 확인 필요'
        lines.append(f"| {p['pair']} {initial} | {w['seed']} | {'재배치' if w['tables']=='shuffle' else '기존 배치'} | "
                     f"{w['movable_objects']} | {marker_ids(b)} / {marker_ids(c)} | "
                     f"{b['score']['elapsed_s']:.1f} / {c['score']['elapsed_s']:.1f} | "
                     f"{int(b['score']['strict'])} / {int(c['score']['strict'])} | {reason} |")
    both=[p for p in data['pairs'] if p['valid'] and not p['repeat']
          and all(p[k]['score']['strict'] for k in ('baseline','candidate'))]
    if both:
        b,c=[mean(p[k]['score']['goal_complete_by_s'] for p in both) for k in ('baseline','candidate')]
        lines+=['',f'양쪽 모두 엄격 완주한 신규 배치 {len(both)}개에서 마지막 확인까지 평균은 Balanced {b:.1f}초, R5 {c:.1f}초였다.']
    if (root/'routes.png').is_file():
        lines+=['','![같은 무작위 시험장에서의 두 정책 경로](routes.png)']
    lines+=['','반복 회차는 독립 배치 수에 더하지 않았다. 이 결과의 범위는 정해진 시험장과 추첨된 배치, 사용한 센서 조건이다.']
    return '\n'.join(lines)+'\n'


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('exam',type=Path);cli.add_argument('output',type=Path)
    args=cli.parse_args();args.output.write_text(report(args.exam))
