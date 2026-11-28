"""Observable motion contracts, using the measured motor ramp/lag as a plant."""
import math
import numpy as np
import pytest
from pinky_soccer_dynamics.params import load
from pinky_soccer_dynamics.profile_model import step
from pinky_soccer_mpc.response_calibration import apply_response
from pinky_soccer_viz.attack_motion import Follower, Grid, wrap
from pinky_soccer_viz.attack_match import RefereeGate


@pytest.mark.parametrize('goal', [(0.05,0,0),(.5,0,0),(0,0,math.pi/2),(0,0,math.pi),(-.3,.1,2.)])
def test_calibrated_plant_arrival_and_no_reentry(goal):
    p,_=apply_response(load('real'))
    f=Follower(p);x=np.zeros(9)
    for i in range(1500):
        u=f.command(x[:3],goal,goal[:2],x[3:5],i*.02)
        x=step(p,x,u,.02)
        if f.done:break
    assert f.done, (goal,x,f.phase)
    assert np.linalg.norm(x[:2]-goal[:2]) < .06
    assert abs(wrap(x[2]-goal[2])) < .12
    for i in range(20):
        assert np.array_equal(f.command(x[:3]+[.01,-.01,.01],goal,goal[:2],x[3:5],31+i*.02),[0,0])


def test_path_detours_and_every_edge_clear():
    g=Grid((-1,-.6,1,.6),.10)
    g.points=np.array([[0,y] for y in np.linspace(-.15,.15,9)])
    path=g.plan([-.6,0],[.6,0])
    assert len(path)>2
    assert all(g.line(a,b) for a,b in zip(path,path[1:]))
    assert max(abs(p[1]) for p in path)>.2


def test_blocked_path_does_not_cut_wall_or_corner():
    g=Grid((-1,-.6,1,.6),.1,blocks=[(-.1,-.6,.1,.6)])
    assert not g.plan([-.5,0],[.5,0])
    assert not g.plan([-.99,0],[.5,0])


def test_referee_restart_and_latched_events():
    gate=RefereeGate()
    assert not gate.can_start(0)
    gate.update(1,True,(.4,0),True);assert not gate.can_start(1)
    gate.update(2,True,(0,0),True);assert gate.can_start(2.1)
    assert not gate.can_start(2.6)
    assert gate.event(1,'OUT',True)=='STOP'
    assert gate.event(1,'OUT',True) is None
    assert gate.event(2,'GOAL',True)=='RETURN'
    assert gate.event(3,'GOAL',False) is None


def test_backup_is_explicit_and_does_not_rotate_to_drive_backwards():
    p,_=apply_response(load('real'));f=Follower(p);x=np.zeros(9);goal=(-.16,0,0)
    for i in range(800):
        u=f.command(x[:3],goal,goal[:2],x[3:5],i*.02,reverse=True)
        x=step(p,x,u,.02)
        if f.done:break
    assert f.done
    assert abs(x[2])<.03
    assert -.2 < x[0] < -.1


def test_unaccepted_terminal_pose_never_claims_done():
    p,_=apply_response(load('real'));f=Follower(p)
    f.command([0,0,0],[0,0,0],[0,0],[0,0],0)
    for i in range(20):
        f.command([.15,0,.3],[0,0,0],[0,0],[0,0],.1+i*.02)
    assert not f.done


def test_static_margin_escape_keeps_physical_clearance_and_rejects_approach():
    from types import SimpleNamespace
    from pinky_soccer_field.field_geometry import Field
    from pinky_soccer_viz.attack_controller import AttackController
    field=Field.from_file('src/pinky_soccer_sim/config/hardboard_field.yaml')
    p,_=apply_response(load('real'))
    n=SimpleNamespace(field=field,params=p,margin=.015,wheels=np.zeros(2),path_mpc=None,
                      profile=np.zeros(2),follower=SimpleNamespace(previous=np.zeros(2)))
    n.static_clearance=lambda pose:AttackController.static_clearance(n,pose)
    n.point_clearance=lambda pose,points:AttackController.point_clearance(n,pose,points)
    pose=(.77,0,.11)
    assert 0<n.static_clearance(pose)<n.margin
    assert AttackController.safe(n,pose,np.array([-1.,-1.]),np.empty((0,2)))
    assert not AttackController.safe(n,pose,np.array([1.,1.]),np.empty((0,2)))
    assert not AttackController.safe(n,(.85,0,0),np.zeros(2),np.empty((0,2)))


def test_contact_approach_reduces_impact_without_global_speed_cap():
    from pinky_soccer_viz.attack_motion import contact_speed_limit
    assert contact_speed_limit((0,0,0),(.5,0),(0,0),.27)==.27
    assert contact_speed_limit((0,0,0),(.1,0),(0,0),.27)==.06
    assert contact_speed_limit((0,0,0),(.1,0),(.24,0),.27)==.27
    assert contact_speed_limit((0,0,0),(.1,0),(-.2,0),.27)==.06


def test_valid_start_does_not_fail_only_because_grid_rounding_hits_obstacle():
    g=Grid((0,0,1,1),.05,resolution=.1)
    g.points=np.array([[.07,.5],[.45,.5]])
    assert g.clear([.14,.5]) and not g.clear([.1,.5])
    path=g.plan([.14,.5],[.8,.5])
    assert path and all(g.line(a,b) for a,b in zip(path,path[1:]))


def test_active_escape_continues_after_it_has_started_making_progress():
    from types import SimpleNamespace
    from pinky_soccer_viz.attack_controller import AttackController
    p,_=apply_response(load('real'));sent=[];reports=[]
    n=SimpleNamespace(now=lambda:2.,progress_at=1.95,recovery_until=2.5,path_mpc=None,
        recovery_command=(-.12,0.),block_reason='',last_progress_warning=2.,
        follower=Follower(p),params=p,wheels=np.zeros(2),profile=np.zeros(2),
        safe=lambda *args:True,send=sent.append,report=lambda *args:reports.append(args),
        stop=lambda reason:pytest.fail('active escape canceled: '+reason))
    AttackController.recover(n,np.zeros(3),np.empty((0,2)),.02,'blocked route')
    assert len(sent)==1 and np.all(sent[0]<0.)
    assert reports[-1][0]=='recover'


@pytest.mark.parametrize('sign', [-1., 1.])
def test_blocked_start_escape_backs_out_without_turning(sign):
    from pinky_soccer_viz.attack_motion import reverse_for_waypoint
    p, _ = apply_response(load('real'))
    pose = np.array([sign*.763, 0., 0. if sign > 0 else math.pi])
    escape = np.array([sign*.523, 0.])
    target = np.array([sign*.616, .277, 0.])
    # This is the first leg of the goal-mouth escape observed in match two.
    reverse = reverse_for_waypoint(pose, escape, True)
    assert reverse
    follower = Follower(p)
    command = follower.command(pose, target, escape, [0., 0.], 0., reverse=reverse)
    assert command[0] < 0 and command[1] < 0
    assert abs(command[0]-command[1]) < 1e-9
    # Normal tactical approaches still use the existing forward-only policy.
    assert not reverse_for_waypoint(pose, escape, False)

def test_path_recovery_prefers_ten_cm_reverse_then_replans():
    from types import SimpleNamespace
    from pinky_soccer_viz.attack_controller import AttackController
    p, _ = apply_response(load('real'))
    sent, stopped = [], []
    clock = [2.]
    n = SimpleNamespace(now=lambda: clock[0], path_mpc=None, progress_at=0.,
        recovery_until=0., recovery_command=None, recovery_attempts=0,
        recovery_travel=0., recovery_backoff_m=.1, recovery_reverse_failed=False,
        block_reason='', last_progress_warning=2., world=SimpleNamespace(our_goal_color='yellow'),
        follower=Follower(p), params=p, margin=.015, wheels=np.zeros(2), profile=np.zeros(2),
        static_clearance=lambda pose: .3, point_clearance=lambda pose, points: .3,
        safe=lambda *args: True, send=sent.append, report=lambda *args: None,
        stop=stopped.append, get_logger=lambda: SimpleNamespace(warning=lambda *args: None))
    AttackController.recover(n, np.zeros(3), np.empty((0,2)), .02, 'no path')
    assert n.recovery_command == (-.12, 0.) and np.all(sent[-1] < 0.)
    n.wheels = np.array([-.12/p.m('left_radius'), -.12/p.m('right_radius')])
    for _ in range(45):
        clock[0] += .02
        AttackController.recover(n, np.zeros(3), np.empty((0,2)), .02, 'no path')
        if stopped:
            break
    assert stopped == ['reverse recovery distance reached; replanning']
    assert .1 <= n.recovery_travel < .103
    assert n.recovery_until == 0 and n.recovery_command is None
    # An unsuccessful rear attempt must select a different motion next time.
    n.recovery_command = (-.12, 0.)
    n.recovery_until = clock[0]-.1
    n.progress_at = clock[0]-2.
    AttackController.recover(n, np.zeros(3), np.empty((0,2)), .02, 'rear escape stalled')
    assert n.recovery_command != (-.12, 0.)
