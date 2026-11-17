"""Apply checked flow changes after the body-scan adapter."""


def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError(f'Unexpected parent source at {old[:100]}')
    return text.replace(old, new)


def apply(package):
    path=package/'osp_mission/mission.py';text=path.read_text()
    text=replace(text, 'from osp_mission.mission_outcome import outcome_status',
        'from osp_mission.mission_outcome import outcome_status\nfrom osp_mission.navigation_flow import NavigationFlow')
    text=replace(text, "'planner_start_ignore_radius_m':0.}",
        "'planner_start_ignore_radius_m':0.,'navigation_flow_enabled':True,'allow_mid_route_wall_inspection':False}")
    text=replace(text, '        self.near_pass=NearPassRuntime(self)',
        "        self.near_pass=NearPassRuntime(self)\n        self.navigation_flow=(NavigationFlow(bool(self.p('allow_mid_route_wall_inspection'))) if self.p('navigation_flow_enabled') else None)")
    text=replace(text, "        generation=self.action_generation\n",
        "        generation=self.action_generation\n        flow=getattr(self,'navigation_flow',None)\n        if flow is not None and kind=='nav':flow.begin_navigation()\n")
    text=replace(text, "            if self.kind=='nav': self.excluded.append(self.destination)",
        "            if self.kind=='nav':\n                self.excluded.append(self.destination)\n                flow=getattr(self,'navigation_flow',None)\n                if flow is not None:flow.record_navigation_result(6,'action_rejected',self.now())")
    text=replace(text, "            if self.kind=='nav':self.excluded.append(self.destination)",
        "            if self.kind=='nav':\n                self.excluded.append(self.destination)\n                flow=getattr(self,'navigation_flow',None)\n                if flow is not None:flow.record_navigation_result(result.status,'bump_contact',self.now())")
    text=replace(text, "        if self.kind=='nav':\n            if getattr(self,'exploration_memory',None)",
        "        if self.kind=='nav':\n            flow=getattr(self,'navigation_flow',None)\n            if flow is not None:flow.record_navigation_result(result.status,getattr(self,'cancel_reason',''),self.now())\n            if getattr(self,'exploration_memory',None)")
    text=replace(text, "        elif self.kind=='spin':\n            self.plan_after=self.now()+1.5",
        "        elif self.kind=='spin':\n            flow=getattr(self,'navigation_flow',None)\n            if flow is None:self.plan_after=self.now()+1.5\n            else:\n                flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.plan_after=self.now()")
    text=replace(text, "        elif self.kind=='backup':\n            self.scan_after=False; self.plan_after=self.now()+1.",
        "        elif self.kind=='backup':\n            flow=getattr(self,'navigation_flow',None)\n            if flow is not None:flow.backup_result(result.status,self.now())\n            self.scan_after=False; self.plan_after=self.now()+1.")
    text=replace(text, "            'camera_diagnostics':getattr(self,'camera_diagnostics',{})})))",
        "            'camera_diagnostics':getattr(self,'camera_diagnostics',{}),\n            'navigation_flow':self.navigation_flow.report() if getattr(self,'navigation_flow',None) is not None else {}})))")
    text=replace(text, "            self.exploration_memory.observe(visit.x,visit.y,self.now())",
        "            self.exploration_memory.observe(visit.x,visit.y,self.now())\n            flow=getattr(self,'navigation_flow',None)\n            if flow is not None and getattr(self,'kind','')=='nav' and (self.goal_handle or self.pending):\n                flow.observe_position(visit.x,visit.y,self.now())")
    text=replace(text, "        if getattr(self,'near_pass',None) is not None:self.near_pass.poll()\n        if self.goal_handle or self.pending:",
        "        flow=getattr(self,'navigation_flow',None)\n        if getattr(self,'near_pass',None) is not None:self.near_pass.poll()\n        if self.goal_handle or self.pending:")
    text=replace(text, "            elif getattr(self,'near_pass',None) is not None:self.near_pass.request()",
        "            elif (flow is not None and self.kind=='nav' and not self.cancel_reason and flow.stalled(self.now())):\n                flow.stalls_cancelled+=1;self.cancel_action('navigation_stalled');self.zero()\n            elif (getattr(self,'near_pass',None) is not None and\n                  (flow is None or flow.allow_mid_route_wall_inspection)):self.near_pass.request()")
    text=replace(text, "                self.plan_after=self.now()+1.5;self.zero();return",
        "                if flow is None:self.plan_after=self.now()+1.5\n                else:flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.plan_after=self.now()\n                self.zero();return")
    text=replace(text, "        if self.now()<self.plan_after: self.zero(); return",
        "        if self.now()<self.plan_after: self.zero(); return\n        if flow is not None and flow.waiting_for_frames(self.now(),self.camera_diagnostics['accepted']):self.zero();return")
    text=replace(text, "self.now()-self.last_recovery>=15.:",
        "self.now()-self.last_recovery>=15. and (flow is None or flow.backup_allowed(self.now())):")
    text=replace(text, "            goal.target_yaw=self.observation_turn()",
        "            protected_look=any(getattr(getattr(self,name,None),'pending',None) is not None for name in ('marker_look','face_look'))\n            goal.target_yaw=self.observation_turn()")
    text=replace(text, "            if abs(goal.target_yaw)<1e-6:\n                self.plan_after=self.now()+1.5; self.zero(); return\n            self.send(self.spin,goal,'spin'); return",
        "            if abs(goal.target_yaw)<1e-6:\n                if flow is None:self.plan_after=self.now()+1.5;self.zero();return\n                if protected_look:\n                    flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.zero();return\n                flow.skipped_zero_turns+=1\n            else:self.send(self.spin,goal,'spin');return")
    text=replace(text, "        selected=(choose_before_deadline(choices,self.weights,",
        "        if flow is not None:choices=flow.filter_goals(choices,memory_transform,self.now())\n        selected=(choose_before_deadline(choices,self.weights,")
    text=replace(text, "        try:\n            save_policy_audit(",
        "        if flow is not None and selected is not None and flow.blocked(selected,memory_transform,self.now()):\n            choices=flow.filter_goals(choices,memory_transform,self.now());selected=self.policy_choose(choices,self.weights)\n        try:\n            save_policy_audit(")
    text=replace(text, "        self.marker_look.select(selected,memory_transform)",
        "        if flow is not None:flow.selected(selected,memory_transform)\n        self.marker_look.select(selected,memory_transform)")
    path.write_text(text)

    path=package/'launch/robot.launch.py';text=path.read_text()
    text=replace(text, "'planner_start_ignore_radius_m':float(value('planner_start_ignore_radius_m')),",
        "'planner_start_ignore_radius_m':float(value('planner_start_ignore_radius_m')),\n            'navigation_flow_enabled':value('navigation_flow_enabled')=='true',\n            'allow_mid_route_wall_inspection':value('allow_mid_route_wall_inspection')=='true',")
    text=replace(text, "'body_scan_filter':'true'}",
        "'body_scan_filter':'true','navigation_flow_enabled':'true','allow_mid_route_wall_inspection':'false'}")
    path.write_text(text)
