"""Exercise failure, retry and archive integrity without starting robot nodes."""
import base64
import io
import json
import hashlib
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest
from types import ModuleType
from unittest.mock import patch

import policy_run_logs as logs


class PolicyLogTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def finished(self,mode='simulation'):
        run=self.root/'run_logs'/mode/'example';run.mkdir(parents=True)
        logs.save(run/'run.json',dict(id='example',mode=mode,state='finished',
            started_utc='2026-11-17T00:00:00+00:00',finished_utc=logs.now(),
            evidence_type='logging_smoke_test',capture_complete=True,auto_upload=True,bag_enabled=False))
        (run/'console.log').write_text('diagnostic output\n')
        return run

    def test_recorder_domain_namespace_and_clock(self):
        robot=logs.bag_command(Path('bag'),'robot','/tb4')
        sim=logs.bag_command(Path('bag'),'simulation')
        self.assertIn('--all',sim);self.assertIn('--use-sim-time',sim)
        self.assertNotIn('--use-sim-time',robot)
        self.assertIn('--include-hidden-topics',robot)
        import re
        expression=robot[robot.index('--regex')+1]
        for topic in ['/tb4/odom','/tb4/oakd/rgb/image','/tf_static','/clock']:
            self.assertIsNotNone(re.match(expression,topic))
        self.assertIsNone(re.match(expression,'/other/odom'))

    def test_archive_parts_reassemble_and_exclude_secrets_and_video(self):
        run=self.finished()
        (run/'rosbag').mkdir();(run/'rosbag/sensor.mcap').write_bytes(bytes(range(256))*50)
        (run/'.env').write_text('SECRET');(run/'credential.key').write_text('SECRET')
        (run/'mission.mp4').write_bytes(b'video');(run/'escape').symlink_to('/etc/passwd')
        (run/'source_snapshot').mkdir();(run/'source_snapshot/large.py').write_text('skip')
        data=logs.package(run,json.loads((run/'run.json').read_text()),limit=100)
        self.assertGreater(len(data['assets']),1)
        combined=b''.join((run/'upload_parts'/a['name']).read_bytes() for a in data['assets'])
        with tarfile.open(fileobj=io.BytesIO(combined),mode='r:gz') as archive:
            names=archive.getnames()
            self.assertIn('run/rosbag/sensor.mcap',names)
            for secret in ['run/.env','run/credential.key','run/escape','run/mission.mp4','run/source_snapshot/large.py']:
                self.assertNotIn(secret,names)
        self.assertEqual(logs.package(run,json.loads((run/'run.json').read_text())),data)
        (run/'upload_parts'/data['assets'][0]['name']).write_bytes(b'changed')
        with self.assertRaises(ValueError):logs.package(run,json.loads((run/'run.json').read_text()))

    def test_child_failure_is_logged_and_upload_failure_is_pending(self):
        with patch.object(logs,'publish',side_effect=RuntimeError('network unavailable')):
            code=logs.run_policy(self.root,'robot',[sys.executable,'-c',
                'print("wheel diagnostic",flush=True);raise SystemExit(17)'],bag=False,validation=True)
        run=next((self.root/'run_logs/robot').iterdir())
        self.assertEqual(code,17)
        self.assertIn('wheel diagnostic',(run/'console.log').read_text())
        self.assertEqual(json.loads((run/'run.json').read_text())['evidence_type'],'logging_smoke_test')
        self.assertEqual(json.loads((run/'upload.json').read_text())['state'],'pending')

    def test_invalid_robot_clock_never_launches(self):
        with self.assertRaises(ValueError):
            logs.run_policy(self.root,'robot',['ros2','launch','x','use_sim_time:=true'])
        self.assertFalse((self.root/'run_logs').exists())

    def test_bag_start_failure_prevents_unrecorded_policy_start(self):
        recorder=unittest.mock.Mock(pid=99999999)
        recorder.poll.return_value=1;recorder.wait.return_value=1
        with patch.object(logs,'git_revision',return_value=None),patch.object(logs.subprocess,'Popen',return_value=recorder) as start,patch.object(logs.time,'sleep'):
            result=logs.run_policy(self.root,'robot',['must-not-start'],bag=True,upload=False)
        self.assertEqual(result,1);self.assertEqual(start.call_count,1)
        manifest=json.loads(next((self.root/'run_logs/robot').glob('*/run.json')).read_text())
        self.assertEqual(manifest['state'],'capture_failed');self.assertFalse(manifest['capture_complete'])

    def test_external_artifacts_are_included_without_raw_git_files(self):
        run=self.finished();external=self.root/'output';external.mkdir()
        (external/'map.yaml').write_text('resolution: 0.025\n')
        manifest=json.loads((run/'run.json').read_text());manifest['artifact_directories']=[str(external)]
        data=logs.package(run,manifest)
        combined=b''.join((run/'upload_parts'/a['name']).read_bytes() for a in data['assets'])
        with tarfile.open(fileobj=io.BytesIO(combined),mode='r:gz') as archive:
            self.assertIn('external_artifacts/0/map.yaml',archive.getnames())

    def test_interrupt_flushes_and_retains_interrupted_state(self):
        proc=subprocess.Popen([sys.executable,str(Path(logs.__file__)),'run','--root',str(self.root),
            '--mode','simulation','--no-bag','--no-upload','--validation-only','--',sys.executable,
            '-c','import time; print("ready",flush=True); time.sleep(30)'],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        try:
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                files=list((self.root/'run_logs/simulation').glob('*/console.log'))
                if files and 'ready' in files[0].read_text():break
                time.sleep(.05)
            else:self.fail('child never became ready')
            proc.send_signal(signal.SIGTERM);self.assertEqual(proc.wait(timeout=10),143)
            manifest=json.loads((files[0].parent/'run.json').read_text())
            self.assertEqual(manifest['state'],'interrupted');self.assertEqual(manifest['signals'],[15])
        finally:
            if proc.poll() is None:proc.kill();proc.wait()

    def fake_github(self,run,corrupt=False):
        remote={};indexes={};calls=[]
        def missing():raise subprocess.CalledProcessError(1,['gh'],stderr='HTTP 404: Not Found')
        def api(args):
            url=args[1]
            if '/releases/tags/' in url:
                calls.append(url);return dict(id=42,html_url='https://github.com/test/release')
            if '/assets?' in url:return [list(remote.values())]
            if '/contents/' in url:
                if '--method' in args:
                    payload=json.loads(Path(args[args.index('--input')+1]).read_text())
                    indexes[url]=payload['content'];return {}
                key=url.removesuffix('?ref=main')
                if key in indexes:return dict(content=indexes[key])
                missing()
            raise AssertionError(args)
        def upload(args,**kwargs):
            self.assertEqual(args[1:3],['release','upload'])
            p=Path(args[4]);remote[p.name]=dict(name=p.name,size=p.stat().st_size,
                digest='sha256:'+('bad' if corrupt else logs.sha(p)),browser_download_url='https://github.com/test/'+p.name)
            return ''
        return api,upload,remote,indexes,calls

    def test_separate_releases_indexes_and_idempotent_verified_upload(self):
        for mode in logs.MODES:
            run=self.finished(mode);api,upload,remote,indexes,calls=self.fake_github(run)
            with patch.object(logs,'gh_json',side_effect=api),patch.object(logs,'command',side_effect=upload):
                logs.publish(run,'test/repo');logs.publish(run,'test/repo')
            self.assertEqual(len(calls),1)
            self.assertIn('policy-logs-'+mode+'-2026-11',calls[0])
            key=next(iter(indexes));self.assertIn('/run_logs/'+mode+'/',key)
            index=json.loads(base64.b64decode(indexes[key]))
            self.assertFalse(index['task_success_claimed'])
            self.assertEqual(json.loads((run/'upload.json').read_text())['state'],'uploaded')

    def test_corrupt_remote_archive_never_marks_uploaded(self):
        run=self.finished();api,upload,*_=self.fake_github(run,corrupt=True)
        with patch.object(logs,'gh_json',side_effect=api),patch.object(logs,'command',side_effect=upload):
            with self.assertRaises(ValueError):logs.publish(run)
        self.assertFalse((run/'upload.json').exists())

    def native(self,report=True):
        native=self.root/'training_robust/native90/test/jobs/00';native.mkdir(parents=True)
        logs.save(native/'manifest.json',dict(environment='gazebo_harmonic'))
        logs.save(native/'service.json',dict(unit='osp-test.service'))
        (native/'service.log').write_text('native console\n')
        if report:logs.save(native/'report.json',dict(rows=[dict(result=dict(success=False))]))
        return native

    def test_native_discovery_defers_active_and_accepts_gone_units(self):
        native=self.native()
        with patch.object(logs.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'LoadState=loaded\nActiveState=active\n','')):
            logs.discover_native(self.root,0)
        self.assertFalse((self.root/'run_logs').exists())
        with patch.object(logs.subprocess,'run',return_value=subprocess.CompletedProcess([],1,'LoadState=not-found\nActiveState=inactive\n','')):
            logs.discover_native(self.root,time.time()+1)
            self.assertFalse((self.root/'run_logs').exists())
            logs.discover_native(self.root,0);logs.discover_native(self.root,0)
        runs=list((self.root/'run_logs/simulation').glob('*/run.json'));self.assertEqual(len(runs),1)
        manifest=json.loads(runs[0].read_text());self.assertEqual(manifest['native_output'],str(native))
        self.assertFalse(manifest['bag_enabled']);self.assertTrue(manifest['capture_complete'])

    def test_native_crash_logs_are_kept_as_incomplete(self):
        self.native(report=False)
        with patch.object(logs.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'LoadState=loaded\nActiveState=failed\n','')):
            logs.discover_native(self.root,0)
        manifest=json.loads(next((self.root/'run_logs/simulation').glob('*/run.json')).read_text())
        self.assertEqual(manifest['state'],'capture_failed');self.assertFalse(manifest['capture_complete'])

    def test_sync_retries_pending_and_marks_orphan_incomplete(self):
        run=self.finished();manifest=json.loads((run/'run.json').read_text())
        manifest.update(state='recording',wrapper_pid=99999999,wrapper_start='1',boot_id=logs.boot_id())
        logs.save(run/'run.json',manifest)
        with patch.object(logs,'publish',side_effect=RuntimeError('offline')) as publish:
            logs.sync(self.root,'test/repo');self.assertEqual(publish.call_count,1)
        self.assertEqual(json.loads((run/'run.json').read_text())['state'],'interrupted_unclean')
        self.assertFalse(json.loads((run/'run.json').read_text())['capture_complete'])
        with patch.object(logs,'publish') as publish:
            logs.sync(self.root,'test/repo');self.assertEqual(publish.call_count,1)

    def test_sync_never_recovers_running_policy(self):
        run=self.finished();manifest=json.loads((run/'run.json').read_text())
        manifest.update(state='recording',wrapper_pid=99999999,wrapper_start='1',boot_id=logs.boot_id())
        logs.save(run/'run.json',manifest)
        with patch.object(logs,'child_group_alive',return_value=True),patch.object(logs,'publish') as publish:
            logs.sync(self.root,'test/repo');publish.assert_not_called()
        self.assertEqual(json.loads((run/'run.json').read_text())['state'],'recording')

    def test_probe_output_option_keeps_generated_files_in_managed_run(self):
        prepared=self.root/'prepared';prepared.mkdir()
        (prepared/'policy.json').write_text('{}');scenario=prepared/'scene.yaml';scenario.write_text('{}')
        digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        logs.save(prepared/'manifest.json',dict(policy_sha256=digest(prepared/'policy.json'),
            scenarios=[dict(file=str(scenario),sha256=digest(scenario),actual_obstacles=0,mount_profile='test')],
            duration=1,planner='edge',camera_width=320))
        parent=ModuleType('osp_mission');parent.__path__=[]
        fake=ModuleType('osp_mission.gazebo_eval')
        fake.one=unittest.mock.Mock(return_value=dict(success=False,reason='synthetic_test',
            runtime_metrics=dict(complete_sensor_coverage=True,stationary_turn_rad_s=dict(samples=10),moving_speed_m_s=dict(samples=10))))
        output=self.root/'isolated/artifacts'
        with patch.dict(sys.modules,{'osp_mission':parent,'osp_mission.gazebo_eval':fake}),patch.object(sys,'argv',
                ['probe','--prepared',str(prepared),'--output',str(output)]):
            runpy.run_path(str(Path(logs.__file__).resolve().parents[1]/'training_robust/run_runtime_probe.py'),run_name='__main__')
        self.assertEqual(fake.one.call_args.args[2],output)
        self.assertTrue((output/'probe_summary.json').is_file())


if __name__=='__main__':unittest.main()
