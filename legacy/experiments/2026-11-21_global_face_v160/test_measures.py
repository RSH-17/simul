import numpy as np
from coverage_metrics import exposure_coverage,surfaces,visible_by_box

def fixture():
    return dict(room_size=[8.,6.],obstacles=[dict(x=.8,y=0.,sx=.72,sy=.72)])

def test_occlusion_respects_ends_and_parallel_outside_segments():
    points=np.array([[.44,0.],[1.16,0.],[1.16,1.5],[-.3,0.]])
    boxes=[(np.array([.44,-.36]),np.array([1.16,.36]))]
    np.testing.assert_equal(visible_by_box(np.array([0.,0.]),points,boxes),[True,False,True,True])

def test_evidence_uses_distinct_processed_exposures_and_correct_face():
    geometry=dict(width=480,height=480,k=[330,0,240,0,330,240,0,0,1])
    mount=dict(translation=[0,0,.45],xyzw=[-.5,.5,-.5,.5])
    times=[0.,.1,.2,.3,.4]
    headings=np.array([[t,-.55,0.,0.] for t in times])
    def score(stamps):
        res=exposure_coverage(fixture(),headings,dict(camera=geometry,base_from_camera=mount,elapsed_s=stamps))
        return res['objects']['0']
    assert score([0.,0.,.05,.08])['complete_faces']==0
    result=score(times)
    assert result['faces']['west']==1. and result['complete_faces']==1
    assert result['faces']['east']==0.

def test_nonsquare_widths_and_wall_denominator():
    scene=fixture();scene['obstacles'][0].update(sx=.8,sy=1.4)
    *_,names,weights,boxes=surfaces(scene)
    assert np.isclose(weights[names=='box0/east'].sum(),1.4)
    assert np.isclose(weights[names=='box0/north'].sum(),.8)
    assert np.isclose(weights[np.char.startswith(names,'wall/')].sum(),2*(7.88+5.88))
