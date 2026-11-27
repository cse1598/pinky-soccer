"""Run without ROS: python3 -m pytest src/pinky_soccer_referee/test."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from referee_core import Rules, Track, associate

FIELD=SimpleNamespace(length=2.,width=1.)
GOALS={-1:("yellow",-.2,.2),1:("blue",-.2,.2)}
R=.02

def test_goal_latches_once_and_reset_preserves_score():
    r=Rules()
    assert r.update([0,0],FIELD,R,GOALS) is None
    assert r.update([1.05,0],FIELD,R,GOALS) is None
    assert r.update([1.06,0],FIELD,R,GOALS) is None
    assert r.update([1.07,0],FIELD,R,GOALS)==("GOAL","blue")
    assert r.update([1.08,0],FIELD,R,GOALS) is None
    assert r.scores["blue"]==1
    r.reset()
    assert r.scores["blue"]==1 and not r.armed

def test_missing_is_not_out():
    r=Rules()
    r.update([0,0],FIELD,R,GOALS)
    for _ in range(10):
        assert r.update(None,FIELD,R,GOALS) is None
    assert r.update([1.2,0],FIELD,R,GOALS) is None
    assert not r.armed

def test_side_out_and_partial_crossing():
    r=Rules()
    r.update([0,0],FIELD,R,GOALS)
    for _ in range(5):
        assert r.update([0,.51],FIELD,R,GOALS) is None
    for _ in range(2):
        assert r.update([0,.53],FIELD,R,GOALS) is None
    assert r.update([0,.54],FIELD,R,GOALS)==("OUT","")

def test_unknown_goal_is_not_goal():
    r=Rules()
    r.update([0,0],FIELD,R,{})
    for _ in range(2):
        assert r.update([1.05,0],FIELD,R,{}) is None
    assert r.update([1.05,0],FIELD,R,{})==("UNVERIFIED_EXIT","")

def test_track_assignment_and_velocity_validity():
    a,b=Track(1),Track(2)
    associate([a,b],[np.array([.6,0]),np.array([-.6,0])],0.)
    assert a.position[0]<0 and b.position[0]>0
    associate([a,b],[np.array([.59,0]),np.array([-.59,0])],.033)
    assert a.velocity_valid and b.velocity_valid
    associate([a,b],[],.066)
    assert not a.visible and not a.velocity_valid
