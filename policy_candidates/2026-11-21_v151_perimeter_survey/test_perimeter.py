import numpy as np
from osp_mission.perimeter_route import PerimeterRoute
from test_survey import setup,choose


def test_object_detour_retains_perimeter_cursor_and_navigation_does_not_credit_camera():
    s,g,c,seen=setup()
    s.survey.update({},g,seen,(0,0,0),[],1)
    choose(s,g,c,seen)
    for f in s.survey.tasks.by_key.values():
        f.state='OBSERVED'
        f.fraction=1.
    goal=choose(s,g,c,seen,40)
    assert 'perimeter_section' in goal
    route=s.survey.perimeter
    route.dispatched(goal)
    before=route.cursor[0].copy()
    key=route.current
    route.dispatched(dict(object_face='1.1'))
    assert route.current==key and np.allclose(route.cursor[0],before)
    assert not route.sections[key].observed
    again=choose(s,g,c,seen,50)
    assert again['perimeter_section']==list(key)


def test_actual_camera_updates_retire_sections_and_failed_path_defers_only_its_section():
    s,g,c,seen=setup()
    choose(s,g,c,seen)
    route=s.survey.perimeter
    assert route.sections and not any(v.observed for v in route.sections.values())
    key=next(iter(route.sections))
    route.dispatched(dict(perimeter_section=list(key)))
    route.outcome(6,'navigation_stalled',10)
    assert route.sections[key].deferred_until>10
    assert sum(v.failures for v in route.sections.values())==1
    seen[:]=True
    choose(s,g,c,seen,20)
    assert any(v.observed for v in route.sections.values())
    assert route.sections[key].failures==1


def test_remaining_time_budget_rejects_unaffordable_wall_goals():
    s,g,c,seen=setup()
    choose(s,g,c,seen)
    route=s.survey.perimeter
    assert route.remaining_estimate_s>0
    assert route.choose(g,c,(1.,1.,0.),(0,0,0),np.full_like(g.cells,1000.,float),
                        np.ones_like(g.cells,float),np.ones_like(c.cells,bool),lambda _:False,599,600) is None
