"""Pure geometry tests; no acados code generation required."""
import numpy as np
from pinky_soccer_mpc.path_time_optimal import corridor_distance, PredictedScene
from pinky_soccer_mpc.obstacle_snapshot import AvoidanceConfig, AvoidanceOptions, Circle, ObstacleSnapshot


def test_corridor_uses_segments_not_only_vertices():
    route=np.array([[0.,0.],[1.,0.],[1.,1.]])
    np.testing.assert_allclose(corridor_distance([[.5,.02],[.9,.5],[1.,1.]],route),[.02,.1,0.])


def test_dynamic_prediction_and_cap():
    s=ObstacleSnapshot(AvoidanceOptions(opponents=True),AvoidanceConfig(),
                       circles=[Circle(.4,0.,.1,'opponent')])
    scene=PredictedScene(s,[[-.2,0.]])
    assert np.isclose(scene.at(.5).circles[0].x,.3)
    assert np.isclose(scene.at(2.).circles[0].x,.2)
    assert np.isclose(scene.at(1.).circles[0].radius,.135)
    poses=np.array([[0.,0.,0.],[0.,0.,0.]])
    c=scene.clearances(poses,[0.,1.])
    assert c[1,0]<c[0,0]
