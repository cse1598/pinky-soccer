"""Smoke checks for the native Nav2 comparison, without a running simulator."""
import math
from types import SimpleNamespace

from pinky_soccer_viz.nav2_stack import nav2_config
from pinky_soccer_viz.nav2_verifier import limit_twist


def test_combined_wheel_limit():
    r, b, max_wheel = .028, .0971, .9*103*2*math.pi/60
    v, w = limit_twist(.25, 2., .25, 2., r, b, max_wheel)
    assert max(abs(v-b*w/2), abs(v+b*w/2))/r <= max_wheel+1e-9
    assert math.isclose(v/w, .25/2.)
    assert limit_twist(float('nan'), 0., .25, 2., r, b, max_wheel) == (0., 0.)


def test_namespaced_nav2_config():
    settings = dict(max_lin=.25, max_ang=2., accel=1.5, ang_accel=10.)
    for name in ('', 'yellow', 'blue'):
        cfg = SimpleNamespace(name=name, simulation=True)
        p = '/' + name if name else ''
        data = nav2_config(cfg, settings)
        controller = data[p+'/controller_server']['ros__parameters']
        assert controller['FollowPath']['motion_model'] == 'DiffDrive'
        assert math.isclose(controller['controller_frequency'] *
                            controller['FollowPath']['model_dt'], 1.)
        assert controller['odom_topic'] == p+'/odom'
        assert controller['goal_checker']['xy_goal_tolerance'] <= .02
        for cm in ('global_costmap', 'local_costmap'):
            costmap = data[p+'/'+cm+'/'+cm]['ros__parameters']
            assert costmap['obstacle_layer']['scan']['topic'] == p+'/scan'
        assert data[p+'/global_costmap/global_costmap']['ros__parameters']['static_layer']['map_topic'] == p+'/map'
