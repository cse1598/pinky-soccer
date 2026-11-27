#!/usr/bin/env python3
"""Temporary observer camera in an isolated Gazebo test; production camera unchanged."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import argparse

import cv2
import numpy as np
import rclpy
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from pinky_interfaces.srv import SetLed
from sensor_msgs.msg import Image as RosImage
from rclpy.qos import qos_profile_sensor_data
from benchmark_localization import LOCAL_DDS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='224',CYCLONEDDS_URI=LOCAL_DDS,
        GZ_PARTITION='pinky_led_'+uuid.uuid4().hex,ROS_LOG_DIR=str(args.output/'ros_logs'))
    log=(args.output/'launch.log').open('w')
    command=['ros2','launch','pinky_soccer_viz','game.launch.py','mode:=gazebo',
             'preparation:=false','verify:=true','headless:=true','field:=hardboard']
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    node=rclpy.create_node('led_render_evaluator')
    client=node.create_client(SetLed,'set_led')
    front=[]
    def on_front(m):
        image=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
        front[:] = [image if m.encoding == 'bgr8' else cv2.cvtColor(image,cv2.COLOR_RGB2BGR)]
    node.create_subscription(RosImage,'camera/image_raw',on_front,qos_profile_sensor_data)
    transport=GzNode()
    images=[]
    def receive(m):
        image=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
        images.append((time.monotonic(),image))
        del images[:-5]
    topic='/validation/led_camera'
    transport.subscribe(Image,topic,receive)
    result={}
    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline and not client.service_is_ready():
            rclpy.spin_once(node,timeout_sec=.1)
        sdf='''<sdf version="1.9"><model name="led_validation_camera"><static>true</static>
        <pose>-.70 0 .13 0 .08 0</pose><link name="camera"><sensor name="observer" type="camera">
        <always_on>true</always_on><update_rate>10</update_rate><topic>/validation/led_camera</topic>
        <camera><horizontal_fov>1.05</horizontal_fov><image><width>640</width><height>480</height>
        <format>R8G8B8</format></image><clip><near>.01</near><far>10</far></clip></camera>
        </sensor></link></model></sdf>'''
        req=EntityFactory(sdf=sdf)
        ok,response=transport.request('/world/hardboard_soccer/create',req,EntityFactory,Boolean,5000)
        result['camera_created']=bool(ok and response.data)
        settle=time.monotonic()+2.
        while time.monotonic()<settle:
            rclpy.spin_once(node,timeout_sec=.05)
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.empty_pb2 import Empty
        ok, scene = transport.request('/world/hardboard_soccer/scene/info', Empty(), Empty, Scene, 1000)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not (ok and any(m.name == 'pinky' for m in scene.model)):
            rclpy.spin_once(node,timeout_sec=.1)
            ok, scene = transport.request('/world/hardboard_soccer/scene/info', Empty(), Empty, Scene, 1000)
        result['scene_models'] = [dict(name=m.name, links=[dict(name=l.name, visuals=[dict(name=v.name,id=v.id) for v in l.visual]) for l in m.link if 'lamp' in l.name]) for m in scene.model] if ok else []
        reference=None
        panels=[]
        for label,color in [('OFF',(0,0,0)),('GREEN',(0,96,0)),('YELLOW',(96,96,0)),
                            ('BLUE',(0,0,96)),('RED',(96,0,0))]:
            future=client.call_async(SetLed.Request(command='fill',r=color[0],g=color[1],b=color[2]))
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and not future.done():
                rclpy.spin_once(node,timeout_sec=.05)
            retry_deadline=time.monotonic()+5
            while (time.monotonic()<retry_deadline and future.done() and
                   not future.result().success):
                rclpy.spin_once(node,timeout_sec=.1)
                future=client.call_async(SetLed.Request(command='fill',r=color[0],g=color[1],b=color[2]))
                deadline=time.monotonic()+1
                while time.monotonic()<deadline and not future.done():
                    rclpy.spin_once(node,timeout_sec=.05)
            after=time.monotonic()
            deadline=after+4
            while time.monotonic()<deadline and (not images or images[-1][0]<after+.5):
                rclpy.spin_once(node,timeout_sec=.05)
            if not images:
                result[label]=dict(image_received=False)
                continue
            image=images[-1][1]
            if reference is None:reference=image.copy()
            result[label]=dict(image_received=True,service_success=bool(future.done() and future.result().success),
                message=future.result().message if future.done() else 'timeout',
                changed_pixels=int(np.count_nonzero(np.any(image!=reference,axis=2))))
            if front:
                hls=cv2.cvtColor(front[-1][130:],cv2.COLOR_BGR2HLS)
                result[label]['front_hls_pixels']={name:int(cv2.countNonZero(cv2.inRange(hls,np.array(low),np.array(high)))) for name,low,high in [('yellow_sim',(24,60,100),(35,180,255)),('blue_sim',(96,20,80),(115,130,255)),('orange',(3,15,100),(19,220,255))]}
            bgr=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            cv2.putText(bgr,label,(10,30),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            cv2.imwrite(str(args.output/(label.lower()+'.png')),bgr)
            panels.append(cv2.resize(bgr,(320,240)))
        if panels:cv2.imwrite(str(args.output/'gazebo_lamp_colors.png'),np.concatenate(panels,axis=1))
    finally:
        transport.unsubscribe(topic)
        transport=None
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
        log.close()
    (args.output/'render.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
