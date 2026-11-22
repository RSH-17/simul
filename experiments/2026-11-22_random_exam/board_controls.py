"""Connect random exam choices to an existing shared 8780 display, preserving legacy runs."""
import argparse
from pathlib import Path
import tempfile

PREVIOUS_MODES=('exam-balanced','exam-print','exam-print-unknown')
FIXED_DESK_GROUP='<optgroup label="실제 시험 · 책상 고정 · 마커 3개">'
NEW_GROUP='<optgroup label="실제 시험 · 고정 책상 · 상자 5개 · 마커 3개">'
STRESS_GROUP='<optgroup label="참고 실험 · 책상 재배치">'
LEGACY_GROUP='<optgroup label="기존 트랙 · 번호 7, 32, 103">'
OLD_GROUP='<optgroup label="시험장 · 장애물과 마커 전체 무작위">'
OFFICIAL=(('exam-official-balanced','Mixed v1.5.1-balanced · 실제 시험'),
          ('exam-official-print','Balanced Print · 실제 시험'),
          ('exam-official-survey','Mixed v1.5.2 Visual Survey · 벽·군집 탐색'))
STRESS=(('exam-balanced','Mixed v1.5.1-balanced · 마커 3개'),
        ('exam-print','Balanced Print · 마커 3개'),
        ('exam-print-unknown','Balanced Print · 마커 1~5개, 수량 미공개'))
OPTIONS=OFFICIAL+STRESS
MODES=tuple(mode for mode,_ in OPTIONS)


def once(source, old, new):
    if source.count(old)!=1:
        raise ValueError('Shared board has a different layout: '+old[:80])
    return source.replace(old,new,1)


def frontend(source):
    def options(items):
        return ''.join(f'<option value="{mode}">{label}</option>' for mode,label in items)
    if FIXED_DESK_GROUP in source:
        source=once(source,FIXED_DESK_GROUP,NEW_GROUP)
    if NEW_GROUP in source:
        if 'value="exam-official-survey"' not in source:
            old=NEW_GROUP+options(OFFICIAL[:-1])+'</optgroup>'
            source=once(source,old,NEW_GROUP+options(OFFICIAL)+'</optgroup>')
        if any(source.count('value="'+mode+'"')!=1 for mode in MODES):
            raise ValueError('Incomplete random exam menu')
        return source
    controls=NEW_GROUP+options(OFFICIAL)+'</optgroup>'+STRESS_GROUP+options(STRESS)+'</optgroup>'+LEGACY_GROUP
    if OLD_GROUP in source:
        return once(source,OLD_GROUP+options(STRESS)+'</optgroup>'+LEGACY_GROUP,controls)
    source=once(source,'트랙 정책 <select><option value="pair">','트랙 정책 <select>'+controls+'<option value="pair">')
    return once(source,'</select></label> <button type="submit">새 랜덤 트랙 주행</button>',
                '</optgroup></select></label> <button type="submit">선택한 정책 주행</button>')


def backend(source):
    anchor_furniture="                        compatible = compatible and layout['randomization'].get('furniture') == ('fixed' if policy.startswith('exam-official-') else 'shuffle')"
    box_filter="""                        bodies = layout.get('obstacles', [])
                        compatible = compatible and layout['randomization'].get('obstacles') == 'exam_boxes'
                        compatible = compatible and len(bodies) == len({b['object_id'] for b in bodies}) == 5
                        compatible = compatible and all(b['name'] == b['object_id'] and b['shape'] == 'box' for b in bodies)"""
    if 'board_run.sh' in source:
        if 'exam-print-unknown' not in source or 'subprocess.Popen(command,' not in source:
            raise ValueError('Incomplete shared board integration')
        if 'exam-official-balanced' not in source:
            source=once(source,' + '+repr(PREVIOUS_MODES)+':',' + '+repr(MODES)+':')
            anchor="                        compatible = compatible and (policy == 'exam-print-unknown' or len(layout['targets']) == 3)"
            source=once(source,anchor,anchor+"\n                        compatible = compatible and layout['randomization'].get('furniture') == ('fixed' if policy.startswith('exam-official-') else 'shuffle')")
        elif 'exam-official-survey' not in source:
            previous=tuple(mode for mode in MODES if mode!='exam-official-survey')
            source=once(source,' + '+repr(previous)+':',' + '+repr(MODES)+':')
        if "'exam_boxes'" not in source:
            source=once(source,anchor_furniture,anchor_furniture+'\n'+box_filter)
        compile(source,'compare_board.py','exec')
        return source
    legacy="('flow','mixed','center','adaptive','shared','r32','pair')"
    source=once(source,'if policy not in '+legacy+':','if policy not in '+legacy+' + '+repr(MODES)+':')
    anchor="            with (args.track_source/'launch-console.log').open('a') as log:"
    dispatch="""            command = [str(args.track_runner), policy]
            if policy.startswith('exam-'):
                import fcntl
                from contextlib import ExitStack
                entry = Path.home()/'random-exam-board/current/experiments/2026-11-22_random_exam/board_run.sh'
                if not entry.is_file():
                    return self.send(503, {'error': '무작위 시험장 실행기를 찾을 수 없습니다.'})
                if reference_dir:
                    try:
                        layout = json.loads((Path(reference_dir)/'scene.json').read_text())
                        compatible = layout.get('randomization', {}).get('generator') == 'exam-random-layout'
                        compatible = compatible and (policy == 'exam-print-unknown' or len(layout['targets']) == 3)
                        compatible = compatible and layout['randomization'].get('furniture') == ('fixed' if policy.startswith('exam-official-') else 'shuffle')
                    except (OSError, KeyError, TypeError, ValueError):
                        compatible = False
                    if not compatible:
                        return self.send(400, {'error': '현재 선택에 맞는 무작위 시험장 회차를 골라 주세요.'})
                try:
                    with ExitStack() as slots:
                        for filename in ('.domain.lock', '.second-domain.lock'):
                            slot = slots.enter_context((args.track_source/filename).open('a'))
                            fcntl.flock(slot, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return self.send(409, {'error': '시뮬레이터에서 다른 주행을 진행 중입니다.'})
                command = ['bash', str(entry), policy]
"""
    dispatch=once(dispatch,anchor_furniture,anchor_furniture+'\n'+box_filter)
    source=once(source,anchor,dispatch+anchor)
    source=once(source,'subprocess.Popen([str(args.track_runner),policy],','subprocess.Popen(command,')
    compile(source,'compare_board.py','exec')
    return source


def install(server, script):
    files=[(server,backend),(script,frontend)]
    originals={p:p.read_bytes() for p,_ in files}
    updates={p:fn(originals[p].decode()).encode() for p,fn in files}
    for p,_ in files:
        if originals[p]==updates[p]:continue
        backup=p.with_name(p.name+'.before_random_exam')
        fn=dict(files)[p]
        if backup.exists() and backup.read_bytes()!=originals[p] and fn(backup.read_text()).encode()!=updates[p]:
            raise ValueError('A different backup already exists: '+str(backup))
        if not backup.exists():backup.write_bytes(originals[p])
        if p.read_bytes()!=originals[p]:raise ValueError('Board file changed during installation')
        with tempfile.NamedTemporaryFile(dir=p.parent,delete=False) as tmp:
            tmp.write(updates[p]);temp=Path(tmp.name)
        temp.chmod(p.stat().st_mode & 0o777);temp.replace(p)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('server',type=Path)
    cli.add_argument('javascript',type=Path)
    args=cli.parse_args()
    install(args.server,args.javascript)
