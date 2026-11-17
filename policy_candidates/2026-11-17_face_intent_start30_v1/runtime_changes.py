"""Apply checked edits to a fresh copy of the frozen face_intent_v2 runtime."""


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Frozen source no longer matches: {old[:100]}')
    return text.replace(old, new, 1)


def apply(package):
    path = package/'osp_mission/edge_policy.py'
    text = path.read_text()
    text = replace_once(text, 'from osp_mission.escape_connectivity import connect_start',
                        'from osp_mission.escape_connectivity import connect_start\nfrom osp_mission.near_start import open_start_region, validate_radius')
    text = replace_once(text, 'inspection_focal=None):', 'inspection_focal=None,start_ignore_radius=0.):')
    text = replace_once(text, '    max_readable=3.7 if inspection_focal is None else readable_distance(inspection_focal)',
                        '    start_ignore_radius=validate_radius(start_ignore_radius)\n    max_readable=3.7 if inspection_focal is None else readable_distance(inspection_focal)')
    text = replace_once(text,
        '    if not (0<=ry<data.shape[0] and 0<=rx<data.shape[1]) or not free[ry,rx]:return [],None\n    safe=connect_start(data,resolution,(ry,rx),safe)',
        '    if not (0<=ry<data.shape[0] and 0<=rx<data.shape[1]):return [],None\n    if start_ignore_radius<=0 and not free[ry,rx]:return [],None\n    safe=(open_start_region(safe,resolution,origin,robot,start_ignore_radius)\n          if start_ignore_radius>0 else connect_start(data,resolution,(ry,rx),safe))')
    text = replace_once(text, '    valid=safe&(clearance>=.32)&np.isfinite(distance)',
                        '    valid=free&safe&(clearance>=.32)&np.isfinite(distance)')
    path.write_text(text)

    path = package/'osp_mission/mission.py'
    text = path.read_text()
    text = replace_once(text, 'from osp_mission.face_directed_look import FaceDirectedLook',
                        'from osp_mission.face_directed_look import FaceDirectedLook\nfrom osp_mission.near_start import validate_radius')
    text = replace_once(text, "'planner':'legacy','require_drive_controller':False}",
                        "'planner':'legacy','require_drive_controller':False,'planner_start_ignore_radius_m':0.}")
    text = replace_once(text, "        self.edge_mode=self.p('planner')=='edge'",
                        "        self.edge_mode=self.p('planner')=='edge'\n        self.planner_start_ignore_radius_m=validate_radius(self.p('planner_start_ignore_radius_m'))")
    text = replace_once(text, "            'sensor_interruptions':getattr(self,'sensor_interruptions',0),",
                        "            'sensor_interruptions':getattr(self,'sensor_interruptions',0),\n            'planner_start_ignore_radius_m':getattr(self,'planner_start_ignore_radius_m',0.),")
    text = replace_once(text, "            option_settings['inspection_focal']=calibration[0] if calibration is not None else 0.",
                        "            option_settings['inspection_focal']=calibration[0] if calibration is not None else 0.\n            option_settings['start_ignore_radius']=getattr(self,'planner_start_ignore_radius_m',0.)")
    path.write_text(text)

    path = package/'launch/robot.launch.py'
    text = path.read_text()
    text = replace_once(text, "'require_drive_controller':value('require_drive_controller')=='true',",
                        "'require_drive_controller':value('require_drive_controller')=='true',\n            'planner_start_ignore_radius_m':float(value('planner_start_ignore_radius_m')),")
    text = replace_once(text, "'require_drive_controller':'false'}",
                        "'require_drive_controller':'false','planner_start_ignore_radius_m':'0.0'}")
    path.write_text(text)
