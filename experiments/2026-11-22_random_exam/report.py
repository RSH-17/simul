"""Write a human-readable paired result directly from the adjudicated ledger."""
import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import mean


def read(path):
    return json.loads(path.read_text())


def marker_ids(row):
    return '·'.join(str(i) for i in sorted(int(i) for i,v in row['score']['records'].items()
                                          if v['verified_s'] is not None)) or '없음'

def candidate_name(root):
    source = root/'registration.json'
    label = read(source)['candidate_label'] if source.is_file() else 'surface-followthrough-r5'
    return {'surface-followthrough-r5': 'R5', 'balanced-print-relook': 'Balanced Print'}.get(label, label)


def layout_results(pairs, worlds, candidate='R5'):
    rows = []
    for furniture, label in (('shuffle', '책상까지 무작위'), ('fixed', '원래 책상 배치')):
        group = [pair for pair in pairs if pair['valid'] and not pair['repeat']
                 and worlds[pair['seed']]['tables'] == furniture]
        for role, policy in (('baseline', 'Balanced'), ('candidate', candidate)):
            scores = [pair[role]['score'] for pair in group]
            if scores:
                total = lambda key: sum(score[key] for score in scores)
                rows.append(f"| {label} | {policy} | {total('strict')}/{len(scores)} | "
                            f"{total('wall_near')}/{total('wall')} | {total('obstacle_near')}/{total('obstacle')} |")
    if not rows:
        return []
    return ['', '### 책상 배치별 첫 주행', '',
            '| 배치 조건 | 정책 | 엄격 완주 | 벽 접근 | 장애물 접근 |',
            '|---|---|---:|---:|---:|', *rows]


def missed_markers(root, pairs, candidate='R5'):
    counts = {'baseline': Counter(), 'candidate': Counter()}
    names = {'registered_without_both_near_witnesses': '등록 후 근거리 증거 부족',
             'decoded_without_registration': '주행 중 번호 판독 후 미등록',
             'predicted_readable_view_without_online_decode': '판독 가능한 기하 조건에서 주행 중 판독 0회',
             'only_a_small_predicted_view': '예상 크기가 작은 시야만 확보',
             'no_clear_predicted_view_at_6px': '6px 이상으로 가림 없이 볼 기회 없음'}
    audited = 0
    for pair in pairs:
        if not pair['valid'] or pair['repeat']:
            continue
        files = {role: root/'camera-opportunities'/(pair[role]['name']+'.json') for role in counts}
        if not all(path.is_file() for path in files.values()):
            continue
        audited += 1
        for role, path in files.items():
            result = read(path)
            if result['run'] != pair[role]['name'] or set(result['targets']) != set(pair[role]['score']['records']):
                raise ValueError('Camera audit and scored marker roster disagree')
            for target in result['targets'].values():
                stage = target['stage']
                if stage != 'verified' and stage not in names:
                    raise ValueError('Unrecognized camera evidence stage: ' + stage)
                counts[role][stage] += 1
    if not audited:
        return []
    return ['', f'### 접근을 마치지 못한 마커 ({audited}개 독립 배치)', '',
            f'| 확인한 마지막 단계 | Balanced | {candidate} |', '|---|---:|---:|',
            *(f"| {label} | {counts['baseline'][stage]} | {counts['candidate'][stage]} |"
              for stage, label in names.items()),
            '', '카메라 보정값, 시뮬레이션 안의 실제 인쇄물·가림과 영상 시각을 대조했다. 기하학적으로 보일 기회와 검출기가 실제로 번호를 읽은 횟수를 구분했다.']


def supplemental(root, pairs, candidate='R5'):
    groups = {}
    for role in ('baseline', 'candidate'):
        groups[role] = []
    for pair in pairs:
        if not pair['valid'] or pair['repeat']:
            continue
        files = {role: root/'sensor-audit'/(pair[role]['name']+'.json') for role in groups}
        if not all(path.is_file() for path in files.values()):
            continue
        records = {role: read(path) for role, path in files.items()}
        if all(record.get('valid') and record.get('surface_coverage') for record in records.values()):
            for role, record in records.items():
                groups[role].append(record)
    n = len(groups['baseline'])
    if not n:
        return []
    lines = ['', f'### 실제 카메라가 처리한 시각의 표면 관찰 기회 ({n}쌍)', '',
             '| 정책 | 벽 관찰 가능 구간 | 장애물 관찰 가능 구간 | 네 면 관찰 기회가 있었던 상자 | Nav2 실패 | Nav2 취소 |',
             '|---|---:|---:|---:|---:|---:|']
    for role, records in groups.items():
        coverage = [row['surface_coverage'] for row in records]
        def percent(kind):
            span = sum(r[kind]['mountable_center_span_m'] for r in coverage)
            return f"{100*sum(r[kind]['observed_span_m'] for r in coverage)/span:.1f}%" if span else '해당 없음'
        lines.append(f"| {'Balanced' if role=='baseline' else candidate} | {percent('wall')} | {percent('obstacle')} | "
                     f"{sum(r['boxes_all_four'] for r in coverage)}/{sum(r['boxes'] for r in coverage)} | "
                     f"{sum(r['navigate']['aborted'] for r in records)} | {sum(r['navigate']['canceled'] for r in records)} |")
    lines.extend(['', '높이 32cm에 20cm 인쇄물을 붙일 수 있는 구간을 최대 40cm 간격으로 표본 조사했다. '
                  '처리된 영상 세 장에서 1초 안에 최단 변 18픽셀, 입사각 60° 이내, 네 모서리와 중심까지 시야 확보를 요구했다. '
                  '직사각 상자는 각 면의 표본 70% 이상을 네 면에서 충족해야 센다. 실제 ID 판독과 근거리 접근은 위 표에서 별도로 채점했다.'])
    return lines


def report(root):
    data=read(root/'comparison.json');protocol=read(root/'protocol.json')
    candidate=candidate_name(root)
    worlds={w['seed']:w for w in protocol['worlds']}
    lines=['# 8780 무작위 시험장 주행 결과','',
           f"등록한 {data['planned_pairs']}쌍 중 {data['paired']}쌍의 양쪽 주행이 끝났다. "
           f"같은 출발 자세와 월드, 온전한 계측을 확인한 쌍은 {data['valid_pairs']}개다.",'',
           '| 처음 주행한 배치 | 엄격 완주 | ID 등록 | 1.5m 접근 | 벽 접근 | 장애물 접근 | 범퍼 | 오등록 | 평균 소요(초) | 평균 거리(m) |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for key,title in (('baseline','Balanced'),('candidate',candidate)):
        r=data['first_runs'][key]
        if r['runs']==0:continue
        lines.append(f"| {title} | {r['strict']}/{r['runs']} | {r['registered']}/{r['all_markers']} | "
                     f"{r['verified']}/{r['all_markers']} | {r['wall_verified']}/{r['walls']} | "
                     f"{r['obstacle_verified']}/{r['obstacles']} | {r['bumps']} | {r['false_registered_instances']} | "
                     f"{r['mean_time_with_failures_at_limit_s']:.1f} | {r['mean_travel_m']:.1f} |")
    lines+=['','시간은 엄격 완주의 마지막 근거리 증거 시각이며 미완주에는 600초를 배정했다. 아래 주행 시간은 실제 시뮬레이션 종료 시각이다.','',
            f'| 배치 | 시드 | 책상 | 이동식 장애물 수 | 실제 등록 후 접근 ID (Balanced / {candidate}) | 종료 시간(초) (Balanced / {candidate}) | 엄격 완주 (Balanced / {candidate}) | 비교쌍 |',
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
        lines+=['',f'양쪽 모두 엄격 완주한 신규 배치 {len(both)}개에서 마지막 확인까지 평균은 Balanced {b:.1f}초, {candidate} {c:.1f}초였다.']
    lines.extend(layout_results(data['pairs'], worlds, candidate))
    lines.extend(missed_markers(root, data['pairs'], candidate))
    lines.extend(supplemental(root, data['pairs'], candidate))
    if (root/'routes.png').is_file():
        lines+=['','![같은 무작위 시험장에서의 두 정책 경로](routes.png)']
    lines+=['','반복 회차는 독립 배치 수에 더하지 않았다. 이 결과의 범위는 정해진 시험장과 추첨된 배치, 사용한 센서 조건이다.']
    return '\n'.join(lines)+'\n'


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('exam',type=Path);cli.add_argument('output',type=Path)
    args=cli.parse_args();args.output.write_text(report(args.exam))
