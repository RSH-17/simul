"""Apply body-only filtering and explicit outcomes after the start30 adapter."""


def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError(f'Unexpected parent source at {old[:90]}')
    return text.replace(old, new)


def apply(package):
    path=package/'osp_mission/mission.py';text=path.read_text()
    text=replace(text, 'from osp_mission.near_start import validate_radius',
        'from osp_mission.near_start import validate_radius\nfrom osp_mission.mission_outcome import outcome_status')
    text=replace(text, "            'elapsed_seconds':max(0.,self.now()-self.started) if self.started else 0.,",
        '            **outcome_status(self),')
    path.write_text(text)
    path=package/'setup.py';text=path.read_text()
    text=replace(text, "'mission = osp_mission.mission:main',",
        "'body_scan_filter = osp_mission.body_scan_filter:main', 'mission = osp_mission.mission:main',")
    path.write_text(text)
    path=package/'launch/robot.launch.py';text=path.read_text()
    text=replace(text, 'from osp_mission.transport_config import image_environment',
        'from osp_mission.transport_config import image_environment\nfrom osp_mission.body_scan_filter import route_costmap_scans')
    text=replace(text, "    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'",
        "    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'\n    raw_scan=value('scan_topic')\n    filtering=not sim and value('body_scan_filter')=='true'\n    planning_scan=('/'+'/'.join(p for p in (value('namespace').strip('/'),'scan_self_filtered') if p)) if filtering else raw_scan\n    filter_actions=([Node(package='osp_mission',executable='body_scan_filter',output='screen',parameters=[{'input_topic':raw_scan,'output_topic':planning_scan,'base_frame':value('base_frame')}])] if filtering else [])")
    text=replace(text, "p.update(scan_topic=value('scan_topic'),", 'p.update(scan_topic=planning_scan,')
    text=replace(text, "    navigation=IncludeLaunchDescription(",
        "    nav2_params=value('nav2_params') or str(pkg/'config/nav2.yaml')\n    if filtering:\n        nav_config=route_costmap_scans(yaml.safe_load(Path(nav2_params).read_text()),raw_scan,planning_scan)\n        nav_dest=Path(tempfile.mkdtemp(prefix='osp_body_scan_'))/'nav2.yaml'\n        nav_dest.write_text(yaml.safe_dump(nav_config));nav2_params=str(nav_dest)\n    navigation=IncludeLaunchDescription(")
    text=replace(text, "'params_file':value('nav2_params') or str(pkg/'config/nav2.yaml')", "'params_file':nav2_params")
    text=replace(text, "        slam_action,\n        Node(package='osp_mission',executable='detector'",
        "        *filter_actions,slam_action,\n        Node(package='osp_mission',executable='detector'")
    text=replace(text, "'scan_topic':value('scan_topic')", "'scan_topic':planning_scan", 3)
    text=replace(text, "'planner_start_ignore_radius_m':'0.0'}",
        "'planner_start_ignore_radius_m':'0.0','body_scan_filter':'true'}")
    path.write_text(text)
