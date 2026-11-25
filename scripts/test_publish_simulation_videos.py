"""Selective release publishing must not upload unrelated completed recordings."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import publish_simulation_videos as publisher


class SelectivePublishingTest(unittest.TestCase):
    def test_random_roster_uses_post_registration_evidence(self):
        self.assertIsNone(publisher.verified_approach({}))
        distances={str(i):1. for i in (0,21,741)}
        self.assertTrue(publisher.verified_approach({'closest_approach_m':distances}))
        record=dict(expected_ids=[0,21,741],verified_after_registration_ids=[0,21],
                    closest_approach_m=distances)
        self.assertFalse(publisher.verified_approach(record))
        self.assertTrue(publisher.verified_approach(dict(expected_ids=[0],verified_after_registration_ids=[0])))
        with self.assertRaises(ValueError):
            publisher.verified_approach(dict(expected_ids=[7,8],verified_after_registration_ids=[7,99]))

    def test_only_requested_video_uploads_and_complete_index_is_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'repository_snapshot').mkdir();(root/'scripts').mkdir()
            (root/'README.md').write_text('Simulation recordings\n')
            videos=[]
            for name in ['requested','other']:
                episode=root/'training_robust/native90'/name/'episode'
                video=episode/'video';video.mkdir(parents=True)
                file=video/'mission.mp4'
                writer=cv2.VideoWriter(str(file),cv2.VideoWriter_fourcc(*'mp4v'),5,(64,48))
                assert writer.isOpened()
                for _ in range(2):writer.write(np.zeros((48,64,3),np.uint8))
                writer.release();videos.append(file)
                (video/'recording.json').write_text(json.dumps({'frames':2,'duration_seconds':.4}))
                (episode/'evaluation.json').write_text(json.dumps({'success':False,'reason':'timeout','targets':1,'collisions':0}))
            (videos[0].parent.parent.parent/'environment_invalidation.json').write_text(json.dumps(
                dict(policy_training_sample_valid=False,reason='Invalid floor sensor input')))
            assets=[];uploaded=[]
            def command(args):
                endpoint=args[-1]
                if endpoint.endswith('/releases/tags/simulation-videos-2026-11'):
                    return json.dumps({'id':1,'html_url':'https://github.com/owner/repo/releases/tag/test'})
                if 'assets?' in endpoint:return json.dumps(assets)
                return json.dumps({'private':False})
            def upload(args,**kwargs):
                path=Path(args[4]);uploaded.append(path.name)
                assets.append({'name':path.name,'size':path.stat().st_size,
                    'digest':'sha256:'+hashlib.sha256(path.read_bytes()).hexdigest()})
            with patch.object(publisher,'__file__',str(root/'scripts/publish_simulation_videos.py')), \
                 patch.object(publisher,'command',side_effect=command), \
                 patch.object(publisher.subprocess,'run',side_effect=upload), \
                 patch('sys.argv',['publish','--upload','--only-upload',str(videos[0])]), \
                 contextlib.redirect_stdout(io.StringIO()):
                publisher.main()
            self.assertEqual(uploaded,['requested__episode.mp4'])
            report=json.loads((root/'repository_snapshot/videos.json').read_text())
            self.assertEqual(len(report['videos']),2)
            self.assertEqual(report['uploaded_verified'],1)
            self.assertFalse(report['repository_private'])
            invalid=next(v for v in report['videos'] if v['asset']=='requested__episode.mp4')
            self.assertFalse(invalid['policy_training_sample_valid'])
            self.assertFalse(invalid['original_evaluator_success'])
            self.assertIn('환경 오류',(root/'repository_snapshot/videos.html').read_text())
            self.assertNotIn('로그인해야',(root/'repository_snapshot/videos.html').read_text())
            isolated=root/'isolated';(isolated/'scripts').mkdir(parents=True)
            (isolated/'repository_snapshot').mkdir();(isolated/'README.md').write_text('Separate checkout\n')
            original_readme=(root/'README.md').read_text()
            with patch.object(publisher,'__file__',str(isolated/'scripts/publish_simulation_videos.py')), \
                 patch.object(publisher,'command',side_effect=command), \
                 patch.object(publisher.subprocess,'run',side_effect=upload), \
                 patch('sys.argv',['publish','--data-root',str(root),'--upload','--only-upload',str(videos[0])]), \
                 contextlib.redirect_stdout(io.StringIO()):
                publisher.main()
            self.assertEqual((root/'README.md').read_text(),original_readme)
            isolated_report=json.loads((isolated/'repository_snapshot/videos.json').read_text())
            self.assertEqual(len(isolated_report['videos']),2)
            self.assertEqual(isolated_report['uploaded_verified'],1)
            self.assertEqual(uploaded,['requested__episode.mp4'])

    def test_verified_entries_in_another_repository_are_kept(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'repository_snapshot').mkdir();(root/'scripts').mkdir()
            (root/'README.md').write_text('Simulation recordings\n')
            old=dict(asset='upstream__episode.mp4',local_path='training_robust/native90/upstream/episode/video/mission.mp4',
                     bytes=1,sha256='0'*64,url='https://github.com/upstream/simul/releases/download/t/upstream__episode.mp4',
                     original_evaluator_success=True,elapsed_sim_seconds=300.0,actual_approach_1_5m=True,uploaded_verified=True)
            pending=dict(old,asset='unverified__episode.mp4',uploaded_verified=False)
            (root/'repository_snapshot/videos.json').write_text(json.dumps({'videos':[old,pending]}))
            def command(args):
                endpoint=args[-1]
                if endpoint.endswith('/releases/tags/simulation-videos-2026-11'):
                    return json.dumps({'id':1,'html_url':'https://github.com/RSH-17/simul/releases/tag/test'})
                if 'assets?' in endpoint:return '[]'
                return json.dumps({'private':True})
            with patch.object(publisher,'__file__',str(root/'scripts/publish_simulation_videos.py')), \
                 patch.object(publisher,'command',side_effect=command), \
                 patch('sys.argv',['publish']), contextlib.redirect_stdout(io.StringIO()):
                publisher.main()
            report=json.loads((root/'repository_snapshot/videos.json').read_text())
            self.assertEqual([v['asset'] for v in report['videos']],['upstream__episode.mp4'])
            self.assertEqual(report['uploaded_verified'],1)
            self.assertIn(old['url'],(root/'README.md').read_text())


if __name__=='__main__':unittest.main()
