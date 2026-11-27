"""A clipped visible arc must reach the policy without inventing a distance fix."""
from types import SimpleNamespace
import json
import cv2
import numpy as np
from pinky_soccer_world_model.robust_ball import observe_ball
from pinky_soccer_world_model.ball_detector import observation_status
from pinky_soccer_world_model.world_model_node import WorldModelNode


def test_bottom_arc_flows_through_world_evidence_without_metric_fix():
    frame=np.zeros((480,640,3),np.uint8)
    cv2.circle(frame,(320,500),35,(4,57,230),-1)
    arc=observe_ball(frame,[])
    assert arc.bearing_valid and not arc.distance_valid
    messages=[]
    node=SimpleNamespace(status_pub=SimpleNamespace(publish=messages.append),rotated=False)
    header=SimpleNamespace(stamp=SimpleNamespace(sec=2,nanosec=0),frame_id='camera')
    observation_status(node,arc,header)
    metric=json.loads(messages[-1].data)
    assert not metric['range_valid']
    wm=SimpleNamespace(ball_input="robot")
    WorldModelNode.on_ball_evidence(wm,messages[-1])
    assert wm.ball_contact=={'stamp':2.,'present':True}
    metric['bearing_valid']=False
    WorldModelNode.on_ball_evidence(wm,SimpleNamespace(data=json.dumps(metric)))
    assert not wm.ball_contact['present']
