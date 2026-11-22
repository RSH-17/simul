"""Connect random exam choices to an existing shared 8780 display, preserving legacy runs."""
import argparse
from pathlib import Path
import tempfile

MODES=('exam-balanced','exam-print','exam-print-unknown')
NEW_GROUP='<optgroup label="시험장 · 장애물과 마커 전체 무작위">'
OPTIONS=(('exam-balanced','Mixed v1.5.1-balanced · 마커 3개'),
         ('exam-print','Balanced Print · 마커 3개'),
         ('exam-print-unknown','Balanced Print · 마커 1~5개, 수량 미공개'))


def once(source, old, new):
    if source.count(old)!=1:
        raise ValueError('Shared board has a different layout: '+old[:80])
    return source.replace(old,new,1)


def frontend(source):
    if NEW_GROUP in source:
        if any(source.count('value="'+mode+'"')!=1 for mode in MODES):
            raise ValueError('Incomplete random exam menu')
        return source
    controls=NEW_GROUP+''.join(f'<option value="{mode}">{label}</option>' for mode,label in OPTIONS)
    controls+='</optgroup><optgroup label="기존 트랙 · 번호 7, 32, 103">'
    source=once(source,'트랙 정책 <select><option value="pair">','트랙 정책 <select>'+controls+'<option value="pair">')
    return once(source,'</select></label> <button type="submit">새 랜덤 트랙 주행</button>',
                '</optgroup></select></label> <button type="submit">선택한 정책 주행</button>')


def backend(source):
    if 'board_run.sh' in source:
        if 'exam-print-unknown' not in source or 'subprocess.Popen(command,' not in source:
            raise ValueError('Incomplete shared board integration')
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
        if backup.exists() and backup.read_bytes()!=originals[p]:
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
