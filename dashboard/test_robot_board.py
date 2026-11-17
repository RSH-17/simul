from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_board import topic
from robot_board_core import map_cell, scan_summary, mission_display


def test_start_cell_distinguishes_occupied_from_narrow_clearance():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = 100
    assert map_cell(grid, .1, (0., 0., 0.), (.25, .25)) == {
        'x': 2, 'y': 2, 'value': 100, 'reason': 'start_not_free'}
    grid[2, 2] = 0
    assert map_cell(grid, .1, (0., 0., 0.), (.25, .25))['reason'] == 'start_free'


def test_start_cell_honors_rotated_map_origin():
    grid = np.zeros((4, 4), dtype=np.int8)
    grid[1, 2] = 100
    assert map_cell(grid, .1, (1., 2., np.pi/2), (.85, 2.25))['reason'] == 'start_not_free'


def test_scan_metrics_require_same_acquisition_stamp():
    def frame(ranges, stamp):
        return SimpleNamespace(ranges=ranges, range_min=.15, range_max=12.,
            header=SimpleNamespace(frame_id='laser', stamp=stamp))
    raw = frame([.19, .3, float('inf')], 1)
    filtered = frame([float('inf'), .3, float('inf')], 1)
    assert scan_summary(raw, filtered)['removed_returns'] == 1
    assert scan_summary(raw, frame(filtered.ranges, 2))['removed_returns'] is None


def test_namespace_and_absolute_topic_selection():
    assert topic('tb4', 'scan') == '/tb4/scan'
    assert topic('tb4', '/scan') == '/scan'


def test_done_with_no_reason_is_not_presented_as_task_success():
    display=mission_display({'state':'done','targets':0})
    assert display['label']=='종료 · 미완료' and display['kind']!='good'


def test_operator_stop_and_timeout_are_distinct_from_completion():
    assert mission_display({'state':'done','termination_reason':'operator_stop','task_success':False})['label']=='주행 중지'
    assert mission_display({'state':'done','termination_reason':'timeout','task_success':False})['label']=='시간 초과'
    assert mission_display({'state':'done','termination_reason':'all_targets_found','task_success':True})['label']=='마커 임무 완료'


def test_saving_results_does_not_turn_a_stop_into_success():
    display=mission_display({'state':'saving','termination_reason':'operator_stop','task_success':False})
    assert '주행 중지' in display['label'] and display['kind']!='good'


def test_safety_supervisor_stop_reports_bump_without_claiming_success():
    display=mission_display({'state':'done','termination_reason':'operator_stop',
                            'task_success':False,'collisions':1})
    assert display['label']=='범퍼 감지 후 중지' and display['kind']!='good'
