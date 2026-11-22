"""Regression tests for explicit start, stale sensors, duplicate and lost RPCs."""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import robot_session as session


class StartSafetyTest(unittest.TestCase):
    def test_stall_recovery_launch_is_explicit(self):
        args = SimpleNamespace(repo=Path('/repo'), duration=600., recover_stall=False)
        original = session.launch_command(args, Path('/trial'))
        args.recover_stall = True
        self.assertEqual(session.launch_command(args, Path('/trial')), original + ['stall_recovery_enabled:=true'])

    def test_forward_045_reaches_only_the_selected_launch(self):
        args = SimpleNamespace(repo=Path('/repo'), duration=600, forward_045=False)
        baseline = session.launch_command(args, Path('/trial'))
        args.forward_045 = True
        self.assertEqual(session.launch_command(args, Path('/trial')), baseline + ['forward_045:=true'])

    def test_upstream_status_separates_discovery_approach_and_stop(self):
        targets = [dict(id=i, x=.1, y=.5, confirmed=i != 8, approached=i in (7, 8)) for i in (7, 8, 103)]
        packet = dict(state='FINISHED', reason='operator_stop', targets=targets, stop_confirmed=True)
        result = session.normalize_r32(packet)
        self.assertEqual(result['approached_ids'], [7])
        self.assertEqual([point[0] for point in result['confirmed_targets']], [7, 103])
        self.assertEqual(result['state'], 'done')
        self.assertFalse(result['task_success'])
        for target in targets:
            target.update(confirmed=True, approached=True)
        self.assertTrue(session.normalize_r32(dict(packet, reason='targets_complete'))['task_success'])
        self.assertEqual(session.normalize_r32(dict(state='READY', readiness='tf_missing_or_stale'))['state'], 'idle')

    def test_upstream_launch_uses_hardware_yaml_and_separate_output(self):
        args = SimpleNamespace(workspace=Path('/ws'), runtime='upstream-r32')
        cmd = session.launch_command(args, Path('/trial'))
        self.assertEqual(cmd[2:4], ['osp_project', 'mission.launch.py'])
        self.assertIn('config:=/ws/install/osp_project/share/osp_project/resources/mission_r32_hardware.yaml', cmd)
        self.assertIn('output_directory:=/trial/upstream', cmd)
        self.assertIn('use_sim_time:=false', cmd)

    def test_side_inspection_is_explicit_and_other_flow_runs_keep_it_disabled(self):
        args = SimpleNamespace(workspace=Path('/ws'), repo=Path('/repo'), duration=600., runtime='flow')
        original = session.launch_command(args, Path('/trial'))
        self.assertIn('allow_mid_route_wall_inspection:=false', original)
        args.side_inspection = True
        selected = session.launch_command(args, Path('/trial'))
        self.assertIn('allow_mid_route_wall_inspection:=true', selected)

    def test_later_verified_stop_clears_stale_unverified_state(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'session.json'
            path.write_text(json.dumps(dict(phase='stop_unverified', runtime='flow',
                                            requested_wall_epoch_s=123., stationary_verified=False)))
            with patch.object(session, 'stop_motion', return_value=True), \
                 patch.object(session, 'owned_alive', return_value=False), \
                 patch.object(session, 'tracked_group_active', return_value=False):
                self.assertEqual(session.stop(SimpleNamespace(), Mock(sensors=Mock(odom=[])), path), 0)
            result = json.loads(path.read_text())
            self.assertEqual(result['phase'], 'stopped')
            self.assertTrue(result['stationary_verified'])

    def test_shared_camera_only_changes_the_detector_image_input(self):
        args = SimpleNamespace(workspace=Path('/ws'), repo=Path('/repo'), duration=600., runtime='flow')
        original = session.launch_command(args, Path('/trial'))
        args.share_camera = True
        changed = session.launch_command(args, Path('/trial'))
        self.assertEqual(changed, original + ['rgb_topic:=' + session.SESSION_RGB])

    def test_zero_velocity_wait_returns_after_a_fresh_stable_second(self):
        io = Mock()
        io.cancel_clients = {}
        io.stop_client.service_is_ready.return_value = False
        io.spin.side_effect = lambda: time.sleep(.05)
        io.sensors.stationary.return_value = True
        started = time.monotonic()
        self.assertTrue(session.stop_motion(io, 3.0))
        elapsed = time.monotonic()-started
        self.assertGreaterEqual(elapsed, 1.)
        self.assertLess(elapsed, 2.5)

    def healthy(self):
        sensors = session.Sensors()
        for name in session.REQUIRED:
            value = [] if name == "hazards" else False if name == "dock" else (0., 0.) if name == "odom" else None
            sensors.update(name, 10., 100. if name in ("scan", "camera", "camera_info", "odom") else None, value)
        return sensors

    def test_real_sensor_clock_mismatch_blocks_start(self):
        sensors = self.healthy()
        self.assertEqual(sensors.reason(10.1, 100.1), "")
        self.assertIn("clock_or_stamp_stale", sensors.reason(10.1, 103.))
        self.assertIn("clock_or_stamp_stale", sensors.reason(10.1, 99.))
        self.assertIn("not_receiving", sensors.reason(12.1, 100.1))

    def test_real_hazards_and_dock_reject_but_backup_limit_does_not(self):
        sensors = self.healthy()
        sensors.update("hazards", 10., value=[0, 5])
        self.assertEqual(sensors.reason(10.1, 100.1), "")
        for hazard in (1, 2, 3, 4):
            sensors.update("hazards", 10., value=[hazard])
            self.assertIn("active_hazard", sensors.reason(10.1, 100.1))
        sensors.update("hazards", 10., value=[])
        sensors.update("dock", 10., value=True)
        self.assertIn("robot_docked", sensors.reason(10.1, 100.1))

    def test_stop_needs_recent_finite_measured_odometry(self):
        sensors = session.Sensors()
        self.assertFalse(sensors.stationary(10.))
        for now in (9.7, 9.8, 9.9):
            sensors.update("odom", now, value=(0., 0.))
        self.assertTrue(sensors.stationary(10.))
        self.assertFalse(sensors.stationary(12.))
        for value in ((0.1, 0.), (0., 0.1), (math.nan, 0.)):
            sensors.update("odom", 10., value=value)
            self.assertFalse(sensors.stationary(10.))

    def test_camera_is_rejected_before_mission_would_reject_it(self):
        sensors = self.healthy()
        sensors.update("camera", 10., 99.2)
        self.assertEqual(sensors.reason(10., 100.), "camera_clock_or_stamp_stale")

    def test_camera_repair_refuses_active_navigation(self):
        io = Mock()
        io.conflicts.return_value = ["mission"]
        with self.assertRaises(RuntimeError):
            session.repair_camera(io, session.StopFlag(), Mock())
        io.node.create_client.assert_not_called()

    def test_single_bump_allows_existing_mission_recovery(self):
        guard = session.BumpRecoveryGuard()
        self.assertEqual(guard.check(0., [1]), "")
        self.assertEqual(guard.check(1., [1]), "")
        self.assertEqual(guard.total, 1)
        self.assertEqual(guard.check(3., []), "")

    def test_held_bump_stops_if_retreat_does_not_clear_it(self):
        guard = session.BumpRecoveryGuard()
        guard.check(0., [1])
        self.assertIn("held", guard.check(10., [1]))

    def test_any_number_of_released_contacts_allows_recovery(self):
        guard = session.BumpRecoveryGuard()
        for index in range(100):
            now = index * 3.
            self.assertEqual(guard.check(now, [1]), "")
            self.assertEqual(guard.check(now+1., []), "")
        self.assertEqual(guard.total, 100)

    def test_cliff_stall_drop_always_stop_even_with_bump(self):
        for hazard in (2, 3, 4):
            self.assertIn("Hardware", session.BumpRecoveryGuard().check(0., [1, hazard]))

    def client(self):
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value.done.return_value = False
        return client

    def test_start_is_immediate_when_ready_without_fixed_dwell(self):
        gate, client = session.StartGate(), self.client()
        self.assertFalse(gate.step(10., False, "", "idle", client, object))
        client.call_async.assert_called_once()
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        self.assertTrue(gate.step(10.01, False, "", "running", client, object))

    def test_stopped_and_stale_inputs_never_send_start(self):
        gate, client = session.StartGate(), self.client()
        with self.assertRaises(InterruptedError):
            gate.step(10., True, "", "idle", client, object)
        gate.step(10., False, "camera_stale", "idle", client, object)
        client.call_async.assert_not_called()

    def test_existing_mission_never_receives_another_start(self):
        gate, client = session.StartGate(), self.client()
        with self.assertRaises(RuntimeError):
            gate.step(10., False, "", "running", client, object)
        client.call_async.assert_not_called()

    def test_transient_camera_delay_does_not_discard_start_response(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        self.assertTrue(gate.step(10.05, False, "camera_clock_or_stamp_stale", "running", client, object))
        client.call_async.assert_called_once()

    def test_lost_start_reply_is_not_retried_and_pending_is_removed(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        gate.step(11., False, "", "running", client, object)
        with self.assertRaises(TimeoutError):
            gate.step(15.1, False, "", "running", client, object)
        gate.cancel(client)
        client.call_async.assert_called_once()
        client.remove_pending_request.assert_called_once_with(client.call_async.return_value)
        client.call_async.return_value.cancel.assert_called_once()

    def test_readiness_rejection_only_retries_after_answer(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=False, message="Waiting for TF")
        gate.step(10.01, False, "", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 1)
        gate.step(10.27, False, "", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 2)

    def test_hazard_while_start_reply_pending_aborts(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        with self.assertRaises(RuntimeError):
            gate.step(10.1, False, "active_hazard_types:1", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 1)

    def test_clock_checks_never_mutate_system_clock(self):
        with patch.object(session.subprocess, "run", side_effect=[
                SimpleNamespace(stdout="yes\n"), SimpleNamespace(stdout="192.168.185.3\n")]) as run:
            session.clock_check("192.168.185.3")
        self.assertTrue(all(call.args[0][1] in ("show", "show-timesync") for call in run.call_args_list))
        for replies in ([SimpleNamespace(stdout="no\n")],
                        [SimpleNamespace(stdout="yes\n"), SimpleNamespace(stdout="wrong\n")]):
            with patch.object(session.subprocess, "run", side_effect=replies), self.assertRaises(RuntimeError):
                session.clock_check("192.168.185.3")

    def test_preflight_never_starts_existing_navigation(self):
        io = Mock()
        io.safety_reason.return_value = ""
        io.conflicts.return_value = ["mission"]
        with self.assertRaisesRegex(RuntimeError, "Existing navigation"):
            session.preflight(io, session.StopFlag(), 1.)
        io.start_client.call_async.assert_not_called()
        io.zero.assert_not_called()

    def test_end_launch_escalates_after_wait_and_targets_group(self):
        process = Mock(pid=1234)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired("launch", 8), None]
        with patch.object(session.os, "killpg") as kill, patch.object(session, "group_active", return_value=False):
            session.end_launch(process)
        self.assertEqual([v.args[1] for v in kill.call_args_list], [session.signal.SIGINT, session.signal.SIGTERM])

    def test_dry_run_uses_prepared_flow_v4_and_never_builds(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(["bash", str(root / "robot.sh"), "dry-run"], capture_output=True, text=True, check=True)
        self.assertIn("robot_session.py start", result.stdout)
        self.assertIn("face_intent_flow_v4_ws", result.stdout)
        self.assertNotIn("colcon", result.stdout)
        result = subprocess.run(["python3", str(root / "scripts/robot_session.py"), "dry-run"], capture_output=True, text=True, check=True)
        command = json.loads(result.stdout)
        self.assertIn("undock:=false", command)
        self.assertIn("use_sim_time:=false", command)


if __name__ == "__main__":
    unittest.main()
