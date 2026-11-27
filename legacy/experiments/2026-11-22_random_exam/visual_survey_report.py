"""Render the final public report using matched scores and verified video release assets."""
import argparse
import json
from pathlib import Path


NAMES={'balanced':'Mixed v1.5.1-balanced','print':'Balanced Print','survey':'Mixed v1.5.2 Visual Survey'}


def report(comp, videos):
    def link(run):
        matches=[v for v in videos if v['asset'].endswith('__'+run+'.mp4') and v['uploaded_verified']]
        if len(matches)!=1:raise ValueError('Expected one verified release asset: '+run)
        return '[영상]('+matches[0]['url']+')'
    def val(number,mult=1.):
        return '미계측' if number is None else f'{number*mult:.1f}'
    def pooled(z):
        rows=['| 정책 | 엄격 완주 | 벽 접근 | 장애물 접근 | 등록 | 제한시간 포함 평균(초) | 평균 거리(m) | 범퍼 | 오등록 |',
              '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
        for role,title in NAMES.items():
            g=z[role]
            rows.append(f"| {title} | {g['strict']}/{g['n']} | {g['wall_near']}/{g['wall']} | {g['object_near']}/{g['objects']} | {g['registered']}/{g['expected']} | {g['mean_capped_s']:.1f} | {g['mean_travel_m']:.1f} | {g['bump']} | {g['false_ids']} |")
        return rows
    five=comp['five_independent']
    words=['# 벽과 겹친 장애물 탐색의 시험장 주행 결과','',
        '8780의 원래 Gazebo 시험장에서 책상 14개와 다리 56개를 고정했다. 각 맵에서 세 정책은 같은 월드와 같은 출발 자세를 사용했다. 카메라는 480×480·5fps, 제한시간은 600초였다. 로봇에는 전체 개수 3개만 알렸다. 엄격 완주는 세 ID 등록, 등록 뒤 실제 1.5m 접근, 근거리 카메라 증거, 저장 위치 오차 25cm 이내와 범퍼·오등록 0회를 함께 요구한다.',
        '','## 다섯 배치의 첫 주행','',*pooled(five),'',
        '미완주는 시간 평균에 600초로 반영했다. 이동거리는 실제 주행 누적값이다. 후보를 설계할 때 앞의 두 배치 영상을 보았으므로 아래의 미사용 세 배치도 별도로 집계했다.',
        '','### 개발 시 사용하지 않은 세 배치','',*pooled(comp['three_untuned']),'',
        '## 카메라 관찰과 되짚은 거리','',
        '시뮬의 실제 노출 시각·차체 자세·카메라 보정값을 마커 부착 가능 표면에 대조했다. 기준 크기 20cm 인쇄물의 네 모서리와 중심이 가리지 않고, 최소 18px로 1초 안에 3회 보인 표면 길이의 비율이다. 관찰 기회와 실제 ID 판독은 각각 집계했다.',
        '','| 정책 | 벽 표면(%) | 장애물 표면(%) | 최근 경로를 되짚은 거리(m/회) | 그중 새 카메라 관찰·ID 없이 되짚은 거리(m/회) |',
        '|---|---:|---:|---:|---:|']
    for role,title in NAMES.items():
        g=five[role]
        show=lambda k,m:val(g[k]['mean'],m)+f" ({g[k]['n']}회)"
        words.append(f"| {title} | {show('wall_fraction',100)} | {show('object_fraction',100)} | {show('repeated_m',1)} | {show('repeated_no_new_view_m',1)} |")
    words+=['','실제 궤적을 25cm마다 샘플링했다. 14~85초 전에 지난 위치에서 48cm 안으로 들어오고, 사이에 3m 이상 이동하며 1.05m 이상 벗어났고, 진행 방향이 동일하거나 반대로 40° 이내인 구간이 75cm 이상 이어져야 되짚은 거리로 셌다. 그 시점의 최근 20초 동안 새 카메라 표면이나 새 ID가 없었던 거리를 별도 집계했다. 좁은 통로의 필수 복귀도 들어갈 수 있다.',
        '','## 배치별 주행과 원본 카메라 영상','',
        '| 시드·구분 | 정책 | 벽 접근 | 장애물 접근 | 종료(초) | 거리(m) | 범퍼 | 원본 카메라 |',
        '|---|---|---:|---:|---:|---:|---:|---|']
    for g in [*comp['maps'],comp['alternate'],*comp['development_only']]:
        labels={'untuned':'미사용','development':'개발','alternate_pose':'대체 출발','distant_development_only':'원거리 단서 단독 진단'}
        roles=['survey'] if g['purpose']=='distant_development_only' else NAMES
        for role in roles:
            v=g[role]
            words.append(f"| {g['seed']} · {labels[g['purpose']]} | {NAMES[role]} | {v['wall_near']}/{v['wall']} | {v['object_near']}/{v['objects']} | {v['elapsed_s']:.1f} | {v['travel_m']:.1f} | {v['bump']} | {link(v['run'])} |")
    overlap=comp['overlap']
    setup=overlap['setup']
    words+=['','[여섯 출발 조건의 실제 궤적](routes.png)','',
        '## 같은 장애물 군집 앞에서 출발한 두 주행','',
        f"시드 {setup['seed']}의 인쇄물과 장애물은 원래 배치를 사용했다. 출발 위치와 방향만 두 물체가 카메라에서 부분적으로 겹치도록 정했다. 실제 장애물 간격은 {setup['gap_m']:.3f}m, 출발점의 격자 여유는 {setup['grid_clearance_m']:.3f}m였다. 이 진단은 다섯 배치의 평균에서 제외했다.",
        '','| 정책 | 두 물체 근처를 지난 거리(m) | 그중 반복(m) | 그중 새 관찰 없는 반복(m) | 전체 접근 | 종료(초) | 범퍼 | 영상 |',
        '|---|---:|---:|---:|---:|---:|---:|---|']
    for role in ('balanced','survey'):
        v=overlap[role]
        route=v['repeat']['recent'] if v['repeat'] else {}
        words.append(f"| {NAMES[role]} | {val(route.get('cluster_visit_m'))} | {val(route.get('cluster_repeat_m'))} | {val(route.get('cluster_repeat_no_new_view_m'))} | {v['near']}/{v['expected']} | {v['elapsed_s']:.1f} | {v['bump']} | {link(v['run'])} |")
    gate=comp['distant_screen']
    words+=['','군집 근처는 서로 다른 두 장애물 각각의 겉면에서 모두 2m 이내인 지점으로 정의했다. [군집 주행 경로](overlap-routes.png) · [출발 전 시야와 조건](../../visual_survey/cluster-start.json)','',
        '## 원거리 인쇄물 단서','',
        f"미사용 세 맵의 기존 여섯 녹화에서 첫 위치 오차 25cm 이내인 단서가 기존 온라인 위치보다 5초 이상 앞선 배치는 {gate['novel_layouts']}/3, 잘못된 첫 위치·ID는 {gate['wrong_first_hints']}건이었다. 사전 조건에 따라 공식 Visual Survey의 원거리 이동 단서는 {'활성화했다' if gate['advance'] else '꺼 두었다'}.",
        '','[전체 지표와 마커별 시각](comparison.json) · [영상 지문](media.json) · [시뮬레이션 보드](http://192.168.252.2:8780/)','']
    return '\n'.join(words)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('comparison','published_index','output'):p.add_argument(name,type=Path)
    a=p.parse_args()
    a.output.write_text(report(json.loads(a.comparison.read_text()),json.loads(a.published_index.read_text())['videos']))
