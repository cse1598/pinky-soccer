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


from referee_core import RobotTrack, associate_robots, robot_observations, trapezoid_points, orient_goals
import cv2
import pytest


def observation(x, y=0., team="yellow", heading=0.):
    return {"p":np.array([x,y]),"bbox":np.array([x,-y,.16,.12]),
            "heading":heading,"team":team,"quality":.9,"length":.16,"width":.12}


def test_robot_reacquires_after_teleport_without_velocity_spike():
    a,b=RobotTrack(1),RobotTrack(2)
    for t in (0.,.033):
        associate_robots([a,b],[observation(-.5),observation(.5,team="blue")],t)
    assert a.visible and b.visible and a.team=="unknown"
    associate_robots([a,b],[observation(-.49),observation(.49,team="blue")],.066)
    assert a.velocity_valid and b.velocity_valid
    associate_robots([a,b],[observation(.2,.25),observation(.48,team="blue")],.099)
    assert not a.visible and b.visible
    associate_robots([a,b],[observation(.21,.25),observation(.47,team="blue")],.132)
    assert a.visible and a.reacquired and not a.velocity_valid
    assert a.team=="unknown" and np.allclose(a.position,[.21,.25])
    associate_robots([a,b],[observation(.22,.25),observation(.46,team="blue")],.165)
    assert a.visible and a.velocity_valid and not a.reacquired


def test_color_change_does_not_break_track():
    a=RobotTrack(1)
    for i in range(7):
        associate_robots([a],[observation(.0)],i*.033)
    assert a.visible and a.team=="yellow"
    for i in range(7,12):
        associate_robots([a],[observation(.01,team="blue")],i*.033)
        assert a.visible and a.velocity_valid
    assert a.team=="blue"
    associate_robots([a],[observation(.01,team="unknown")],12*.033)
    assert a.visible and a.team=="unknown" and a.identity_team=="unknown"


def test_wraparound_angular_velocity():
    a=RobotTrack(1)
    a.observe(observation(0.,heading=np.deg2rad(179.)),0.,False,.9)
    a.observe(observation(.01,heading=np.deg2rad(-179.)),.1,True,.9)
    assert a.angular_velocity is not None and 0.<a.angular_velocity<np.deg2rad(30.)
    assert abs(abs(a.heading)-np.pi)<np.deg2rad(3.)


class FakeField:
    ppm=550.
    def metric(self,p):
        return np.asarray(p)*np.array([1.,-1.])/self.ppm
    def pixel(self,p):
        return np.round(np.asarray(p)*np.array([1.,-1.])*self.ppm).astype(int)


@pytest.mark.parametrize("led,team",[
    ((220,220,220),"unknown"),((30,220,240),"yellow"),((255,170,30),"blue")])
def test_synthetic_rear_led_vs_blue_wheels(led,team):
    img=np.full((220,300,3),(70,100,70),np.uint8)
    cv2.rectangle(img,(100,80),(190,140),(22,22,22),-1)
    cv2.rectangle(img,(96,91),(107,129),led,-1)
    # Blue wheels are bright on purpose; their long axis runs along the chassis.
    cv2.rectangle(img,(135,71),(165,80),(255,170,30),-1)
    cv2.rectangle(img,(135,140),(165,149),(255,170,30),-1)
    result=robot_observations(img,FakeField(),[(None,5400,(100,80,91,61))])
    assert len(result)==1
    assert result[0]["team"]==team
    assert result[0]["heading"] is not None
    assert abs(result[0]["heading"])<.15


def test_blue_wheels_alone_do_not_assign_team():
    img=np.full((220,300,3),(70,100,70),np.uint8)
    cv2.rectangle(img,(100,80),(190,140),(22,22,22),-1)
    cv2.rectangle(img,(135,71),(165,80),(255,170,30),-1)
    cv2.rectangle(img,(135,140),(165,149),(255,170,30),-1)
    result=robot_observations(img,FakeField(),[(None,5400,(100,80,91,61))])
    assert result[0]["team"]=="unknown"
    assert result[0]["heading"] is None


def test_trapezoid_front_edge_shorter():
    a=RobotTrack(1)
    a.observe(observation(0.),0.,False,.8)
    p=trapezoid_points(a,FakeField())
    assert np.linalg.norm(p[0]-p[1])<np.linalg.norm(p[2]-p[3])
    assert np.mean(p[:2,0])>np.mean(p[2:,0])


def test_goal_orientation_rotates_geometry(monkeypatch):
    import referee_core as core
    class F:
        rotated=False
        def warp(self,img):
            return img
        def rotate_180(self):
            self.rotated=True
    f=F()
    monkeypatch.setattr(core,"goal_intervals",lambda img,field:{-1:("blue",-.25,.15),1:("yellow",-.1,.2)})
    goals=orient_goals(f,None)
    assert f.rotated and goals[-1]==("yellow",-.2,.1) and goals[1]==("blue",-.15,.25)


@pytest.mark.parametrize("bgr,team",[
    ((220,220,220),"unknown"),((150,195,205),"unknown"),
    ((185,220,245),"unknown"),((30,220,240),"yellow"),
    ((255,170,30),"blue"),((170,135,50),"unknown")])
def test_led_interior_color_evidence(bgr,team):
    from referee_core import classify_led_pixels
    pixels=np.tile(np.array(bgr,np.uint8),(100,1))
    assert classify_led_pixels(pixels)==team


def test_small_yellow_reflection_does_not_define_team():
    from referee_core import classify_led_pixels
    pixels=np.tile(np.array([220,220,220],np.uint8),(100,1))
    pixels[:24]=[30,220,240]
    assert classify_led_pixels(pixels)=="unknown"


def test_floor_tape_without_led_is_not_heading():
    img=np.full((220,300,3),(70,100,70),np.uint8)
    cv2.rectangle(img,(96,0),(105,219),(220,220,220),-1)
    cv2.rectangle(img,(100,80),(190,140),(22,22,22),-1)
    result=robot_observations(img,FakeField(),[(None,5400,(100,80,91,61))])
    assert result[0]["heading"] is None and result[0]["team"]=="unknown"


def test_heading_dropout_is_predicted_then_invalidated():
    a=RobotTrack(1)
    a.observe(observation(0.,heading=np.pi/4),0.,False,.9)
    a.observe(observation(0.,heading=np.pi/4),.033,True,.9)
    assert a.heading_measured
    a.observe(observation(0.,heading=None),.066,True,.9)
    assert not a.heading_measured and a.heading is not None
    assert abs(a.heading-np.pi/4)<.01
    assert a.angular_velocity is None
    for t in (.1,.2,.3):
        a.observe(observation(0.,heading=None),t,True,.9)
    a.observe(observation(0.,heading=None),.34,True,.9)
    assert a.heading is None


def test_relocation_resets_heading_and_filter():
    a=RobotTrack(1)
    a.observe(observation(0.,heading=np.pi/4),0.,False,.9)
    a.observe(observation(.01,heading=np.pi/4),.033,True,.9)
    a.observe(observation(.7,heading=None),.1,False,.9)
    assert a.heading is None and not a.velocity_valid
    assert np.allclose(a.position,[.7,0.])


def test_static_kalman_filter_reduces_jitter():
    from referee_core import ConstantVelocityFilter
    rng=np.random.default_rng(37)
    z=rng.normal(0,.003,(100,2))
    f=ConstantVelocityFilter(z[0],.003,2.5)
    filtered=np.array([f.step(v,1/30)[0] for v in z[1:]])
    assert np.std(filtered[20:])<np.std(z[21:])


def test_geometry_rejects_bad_rectification():
    from referee_core import verify_rectified_boundary
    class Rectified:
        length=1.67
        width=1.07
        ppm=550.
        def pixel(self,p):
            return np.rint(np.array([550,350])+np.asarray(p)*[550,-550]).astype(int)
    f=Rectified()
    img=np.full((700,1100,3),(70,100,70),np.uint8)
    a=f.pixel((-.835,.535)); b=f.pixel((.835,-.535))
    cv2.rectangle(img,tuple(a),tuple(b),(220,220,220),7)
    assert max(verify_rectified_boundary(img,f))<=5.
    shifted=np.roll(img,15,axis=0)
    with pytest.raises(ValueError):
        verify_rectified_boundary(shifted,f)


def test_goal_strip_rejects_long_color_artifact():
    from referee_core import goal_intervals
    class Rectified:
        length=1.67
        width=1.07
        ppm=550.
        def pixel(self,p):
            return np.rint(np.array([600,350])+np.asarray(p)*[550,-550]).astype(int)
        def metric(self,p):
            return (np.asarray(p)-[600,350])*[1.,-1.]/550.
    f=Rectified()
    img=np.full((700,1200,3),(70,100,70),np.uint8)
    left=f.pixel((-.835,0))[0]; right=f.pixel((.835,0))[0]
    cv2.rectangle(img,(left-55,240),(left-8,460),(30,220,240),-1)
    cv2.rectangle(img,(right+8,240),(right+55,460),(180,100,40),-1)
    cv2.line(img,(right+10,150),(right+10,550),(180,100,40),2)
    goals=goal_intervals(img,f)
    assert goals[-1][0]=="yellow" and goals[1][0]=="blue"
    assert .37<goals[1][2]-goals[1][1]<.43
