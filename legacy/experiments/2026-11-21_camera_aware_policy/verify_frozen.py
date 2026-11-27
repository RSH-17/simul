"""Read-only checksum audit of the original and tested camera policy stages."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(repository,baseline,selected):
    root=repository/'policy_candidates/2026-11-21_mixed_exposure_sync'
    integrity=json.loads((repository/'experiments/2026-11-20_v12_camera_rate/source_integrity.json').read_text())['sha256']
    unchanged=json.loads((root/'baseline_sha256.json').read_text())
    overlay=json.loads((root/'runtime_sha256.json').read_text())
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for relative,expected in integrity.items():
        assert digest(baseline/'src/osp_mission'/relative)==expected,relative
    for relative,expected in unchanged.items():
        assert digest(baseline/'src/osp_mission'/relative)==expected,relative
        assert digest(selected/'src/osp_mission'/relative)==expected,relative
    for relative,expected in overlay.items():
        assert digest(root/'overlay'/relative)==expected,relative
        assert digest(selected/'src/osp_mission'/relative)==expected,relative
    return dict(original_unchanged_checked_files=len(integrity),
                selected_unchanged_original_files=len(unchanged),exact_selected_overlay_files=len(overlay),
                selected_snapshot_source_files=len(unchanged)+len(overlay),all_sha256_match=True,
                physical_drive_started=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('repository',type=Path)
    parser.add_argument('baseline',type=Path);parser.add_argument('selected',type=Path)
    args=parser.parse_args();print(json.dumps(verify(args.repository,args.baseline,args.selected),indent=2))
