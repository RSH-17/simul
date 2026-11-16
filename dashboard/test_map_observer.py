import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('dashboard_map',Path(__file__).with_name('map_observer.py'))
maps=importlib.util.module_from_spec(spec);spec.loader.exec_module(maps)


def test_map_origin_rotation_keeps_global_path_on_same_cells():
    grid=np.zeros((20,30),dtype=np.int8);grid[3:6,7:9]=100
    plans={'global':dict(sequence=1,points=[[.5,.5],[1.,.5]])}
    first=maps.render_map(grid,.1,(0.,0.,0.),plans)
    transformed={'global':dict(sequence=1,points=maps.MapObserver.project(plans['global']['points'],(4.,-3.,np.pi/2)))}
    second=maps.render_map(grid,.1,(4.,-3.,np.pi/2),transformed)
    assert np.array_equal(first,second)
    assert np.array_equal(first[719,719],maps.BLUE)


def test_every_plan_publication_replaces_previous_and_empty_clears(tmp_path):
    observer=maps.MapObserver.__new__(maps.MapObserver)
    observer.output=tmp_path;observer.counts={};observer.plans={};observer.errors={};calls=[]
    observer.render=lambda:calls.append(True)
    def message(x):
        return SimpleNamespace(header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(sec=12,nanosec=0)),
            poses=[SimpleNamespace(header=SimpleNamespace(frame_id='map'),pose=SimpleNamespace(position=SimpleNamespace(x=x,y=1.)))])
    observer.plan(message(1.),'global');observer.plan(message(2.),'global')
    assert observer.plans['global']['sequence']==2
    assert observer.plans['global']['points']==[[2.,1.]]
    empty=message(0.);empty.poses=[];observer.plan(empty,'global')
    assert observer.plans['global']['points']==[] and len(calls)==3
    assert len((tmp_path/'plans.jsonl').read_text().splitlines())==3


def test_untransformable_frame_never_shows_raw_coordinates_as_map(tmp_path):
    observer=maps.MapObserver.__new__(maps.MapObserver)
    observer.output=tmp_path;observer.counts={};observer.plans={'global':{'points':[[1.,1.]]}};observer.errors={}
    def fail(*args):raise ValueError('TF unavailable')
    observer.transform=fail;observer.render=lambda:None
    message=SimpleNamespace(header=SimpleNamespace(frame_id='odom',stamp=SimpleNamespace(sec=1,nanosec=0)),
        poses=[SimpleNamespace(header=SimpleNamespace(frame_id='odom'),pose=SimpleNamespace(position=SimpleNamespace(x=2.,y=3.)))])
    observer.plan(message,'global')
    assert 'global' not in observer.plans and 'TF unavailable' in observer.errors['global']
