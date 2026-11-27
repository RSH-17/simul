from audit_navigation_resume_budget import analyze


def test_completed_inspections_preserve_target_but_consumed_is_not_exact_arrival():
    faces=[dict(event='selected',stamp=44.5),dict(event='consumed',stamp=147.)]
    inspections=[dict(event='requested',stamp=48.502),dict(event='finished',stamp=52.,resumed_navigation=True),
        dict(event='requested',stamp=99.),dict(event='finished',stamp=102.,resumed_navigation=True)]
    result=analyze(faces,inspections,0)[0]
    assert result['bracket_s']==102.5
    assert 91.9<result['navigation_active_lower_bound_s']<92.1
    assert result['exceeds_original_65_s_allowance']
    assert result['navigation_active_lower_bound_s']<102.5


def test_unknown_sensor_pause_cannot_be_labeled_as_active_navigation():
    result=analyze([dict(event='selected',stamp=0.),dict(event='consumed',stamp=100.)],[],1)[0]
    assert result['navigation_active_lower_bound_s'] is None
    assert not result['exceeds_original_65_s_allowance']


def test_cleared_and_still_pending_faces_do_not_become_completed_intervals():
    faces=[dict(event='selected',stamp=0.),dict(event='cleared',stamp=20.),
        dict(event='consumed',stamp=100.),dict(event='selected',stamp=101.)]
    assert analyze(faces,[],0)==[]
