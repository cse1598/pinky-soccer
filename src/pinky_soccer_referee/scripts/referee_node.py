#!/usr/bin/env python3
"""ROS 2 Jazzy overhead referee. Does not command robots or publish map TF."""
import math
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from pinky_soccer_referee.msg import TrackedObject, RefereeState, RefereeEvent
from pinky_soccer_referee.srv import GetState
from referee_core import Field, Track, Rules, detect, goal_intervals, associate


class Referee(Node):
    def __init__(self):
        super().__init__("referee", namespace="referee")
        defaults = {
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "video_path": "", "show_ui": False, "publish_image": True,
            "field_length_m": 0., "field_width_m": 0.,
            "pixels_per_meter": 550., "margin_m": .15,
            "ball_radius_m": .02135, "warmup_frames": 30,
            "require_camera_info": False, "allow_approximate_rules": False,
            "yellow_goal_y_min": 0., "yellow_goal_y_max": 0.,
            "blue_goal_y_min": 0., "blue_goal_y_max": 0.,
            "yellow_goal_side": -1, "blue_goal_side": 1,
            "frame_timeout_s": .5,
        }
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.params = {k:self.get_parameter(k).value for k in defaults}
        p=self.params
        if min(p["field_length_m"],p["field_width_m"],p["pixels_per_meter"],p["ball_radius_m"],p["margin_m"],p["frame_timeout_s"]) <= 0:
            raise ValueError("Set positive measured white-line field_length_m and field_width_m; see README")
        if p["yellow_goal_side"] not in (-1,1) or p["blue_goal_side"] != -p["yellow_goal_side"]:
            raise ValueError("Goal sides must be opposite and each +/-1")
        self.bridge=CvBridge()
        self.field=None
        self.camera=None
        self.goals={}
        self.rules=Rules()
        self.ball=Track(0)
        self.robots=[Track(1),Track(2)]
        self.frames=0
        self.last_arrival=None
        self.stale=True
        self.last_stamp=None
        self.last_state=RefereeState()
        self.last_state.status="WAIT_FOR_CALIBRATION"
        self.last_state.header.frame_id="referee_field"
        self.last_state.ball=self.msg(self.ball)
        self.last_state.robots=[self.msg(tr) for tr in self.robots]
        self.rule_enabled=False
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub=self.create_publisher(RefereeState,"state",qos)
        self.event_pub=self.create_publisher(RefereeEvent,"events",qos)
        self.ball_pub=self.create_publisher(TrackedObject,"ball",10)
        self.robot_pubs=[self.create_publisher(TrackedObject,"robot_"+str(i),10) for i in (1,2)]
        self.image_pub=self.create_publisher(Image,"top_view",qos_profile_sensor_data)
        self.create_service(GetState,"get_state",self.get_state)
        self.create_service(Trigger,"is_out",self.is_out)
        self.create_service(Trigger,"is_goal",self.is_goal)
        self.create_service(Trigger,"reset_play",self.reset_play)
        self.create_service(Trigger,"reset_score",self.reset_score)
        self.create_service(Trigger,"recalibrate",self.recalibrate)
        self.state_pub.publish(self.last_state)
        self.create_timer(.2,self.watchdog)
        self.cap=None
        if p["video_path"]:
            self.cap=cv2.VideoCapture(p["video_path"])
            if not self.cap.isOpened():
                raise RuntimeError("Cannot open video: "+p["video_path"])
            self.fps=self.cap.get(cv2.CAP_PROP_FPS) or 30.
            if not math.isfinite(self.fps) or self.fps<=0:
                self.fps=30.
            self.video_index=0
            self.video_timer=self.create_timer(1./self.fps,self.read_video)
        else:
            self.create_subscription(CameraInfo,p["camera_info_topic"],self.info,qos_profile_sensor_data)
            self.create_subscription(Image,p["image_topic"],self.on_image,qos_profile_sensor_data)
        self.get_logger().info("Read-only referee ready. No command or TF publisher.")

    def info(self,msg):
        k=np.array(msg.k,dtype=float).reshape(3,3)
        d=np.array(msg.d,dtype=float)
        if msg.distortion_model not in ("plumb_bob","rational_polynomial") or len(d) not in (4,5,8,12,14) or min(k[0,0],k[1,1])<=0 or not np.isfinite(k).all() or not np.isfinite(d).all():
            return
        self.camera=(k,d,int(msg.width),int(msg.height))

    def read_video(self):
        ok,image=self.cap.read()
        if not ok:
            self.video_timer.cancel()
            self.get_logger().info("End of video")
            return
        # Use media time for differentiation and node clock for message headers.
        t=self.video_index/self.fps
        self.video_index+=1
        self.process(image,self.get_clock().now().to_msg(),t)

    def on_image(self,msg):
        try:
            image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning("Image conversion failed: "+str(exc),throttle_duration_sec=2.)
            return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        stamp=msg.header.stamp
        if t==0:
            now=self.get_clock().now()
            t=now.nanoseconds*1e-9
            stamp=now.to_msg()
        self.process(image,stamp,t)

    def calibrate(self,image):
        p=self.params
        camera=None
        if self.camera is not None and self.camera[2:]==(image.shape[1],image.shape[0]):
            camera=self.camera[:2]
        if p["require_camera_info"] and camera is None:
            raise ValueError("Matching CameraInfo required")
        field=Field(image,p["field_length_m"],p["field_width_m"],p["pixels_per_meter"],p["margin_m"],camera)
        top=field.warp(image)
        goals=goal_intervals(top,field)
        measured=True
        for color in ("yellow","blue"):
            lo=p[color+"_goal_y_min"]; hi=p[color+"_goal_y_max"]
            side=p[color+"_goal_side"]
            if hi>lo and -field.width/2<=lo<hi<=field.width/2:
                goals[side]=(color,lo,hi)
            else:
                measured=False
        self.field=field
        self.input_shape=image.shape[:2]
        self.goals=goals
        # Approximate measurements are not competition-certified. Explicit opt-in.
        self.rule_enabled=bool((measured and field.source=="camera_info") or p["allow_approximate_rules"])
        self.get_logger().info("Calibrated: "+field.source+"; goal intervals="+str(goals)+"; rules enabled="+str(self.rule_enabled))

    @staticmethod
    def msg(tr):
        m=TrackedObject()
        m.id=tr.id
        m.visible=tr.visible
        m.velocity_valid=tr.velocity_valid
        m.heading_valid=False
        m.heading=float("nan")
        m.angular_velocity=float("nan")
        m.position.x,m.position.y=(map(float,tr.position) if tr.visible and tr.position is not None else (float("nan"),float("nan")))
        m.position.z=0.
        m.velocity.x,m.velocity.y=(map(float,tr.velocity) if tr.velocity_valid else (float("nan"),float("nan")))
        m.velocity.z=0.
        return m

    def publish(self,stamp,status=None):
        s=RefereeState()
        s.header.stamp=stamp
        s.header.frame_id="referee_field"
        s.calibrated=self.field is not None
        # Metric estimates based on user-provided dimensions, not certified accuracy.
        s.metric_valid=s.calibrated and not self.stale
        s.armed=self.rules.armed and self.rule_enabled and not self.stale
        s.status=status or (self.rules.status if self.rule_enabled else "RULES_DISABLED")
        s.last_event=self.rules.last_event
        s.yellow_goal_entries=self.rules.scores["yellow"]
        s.blue_goal_entries=self.rules.scores["blue"]
        s.ball=self.msg(self.ball)
        s.robots=[self.msg(tr) for tr in self.robots]
        s.ball.header=s.header
        for obj in s.robots:
            obj.header=s.header
        self.last_state=s
        self.state_pub.publish(s)
        self.ball_pub.publish(s.ball)
        for pub,msg in zip(self.robot_pubs,s.robots):
            pub.publish(msg)

    def process(self,image,stamp,t):
        self.last_arrival=time.monotonic()
        self.stale=False
        if self.last_stamp is not None and t<=self.last_stamp:
            self.ball.miss()
            for tr in self.robots:
                tr.miss()
            self.rules.previous=None
            self.rules.pending=None
            self.rules.count=0
        self.last_stamp=t
        self.frames+=1
        if self.field is not None and image.shape[:2]!=self.input_shape:
            self.clear_calibration()
        if self.field is None:
            if self.frames < self.params["warmup_frames"]:
                return
            if self.frames % 10 != 0:
                return
            try:
                self.calibrate(image)
            except Exception as exc:
                self.get_logger().warning("Calibration failed: "+str(exc),throttle_duration_sec=2.)
                self.publish(stamp,"WAIT_FOR_CALIBRATION")
                return
        f=self.field
        top=f.warp(image)
        bs,rs=detect(top,f,self.params["ball_radius_m"])
        associate([self.ball],[f.metric(v[0]) for v in bs[:4]],t,gate=.45)
        associate(self.robots,[f.metric(v[0]) for v in rs[:5]],t)
        # A gap in observations invalidates the candidate; never infer a goal from disappearance.
        event=self.rules.update(self.ball.position if self.ball.visible else None,f,self.params["ball_radius_m"],self.goals) if self.rule_enabled else None
        if event is not None:
            msg=RefereeEvent()
            msg.header.stamp=stamp
            msg.header.frame_id="referee_field"
            msg.sequence=self.rules.sequence
            msg.event,msg.goal_color=event
            msg.ball_position.x,msg.ball_position.y=map(float,self.ball.position)
            self.event_pub.publish(msg)
        self.publish(stamp)
        if self.params["show_ui"] or (self.params["publish_image"] and self.image_pub.get_subscription_count()>0):
            self.draw(top,stamp)

    def draw(self,top,stamp):
        f=self.field
        l,w=f.length/2,f.width/2
        cv2.rectangle(top,tuple(f.pixel((-l,w))),tuple(f.pixel((l,-w))),(255,255,0),2)
        cv2.drawMarker(top,tuple(f.pixel((0.,0.))),(255,255,255),cv2.MARKER_CROSS,14,2)
        for side,(color,lo,hi) in self.goals.items():
            cv2.line(top,tuple(f.pixel((side*l,lo))),tuple(f.pixel((side*l,hi))),
                     (0,230,255) if color=="yellow" else (255,110,0),5)
        for tr in [self.ball]+self.robots:
            if not tr.visible:
                continue
            center=f.pixel(tr.position)
            color=(0,130,255) if tr.id==0 else (0,255,70)
            cv2.circle(top,tuple(center),7,color,2)
            if tr.velocity_valid:
                cv2.arrowedLine(top,tuple(center),tuple(f.pixel(tr.position+.30*tr.velocity)),color,2)
            label=("Ball" if tr.id==0 else "Track "+str(tr.id))+" %.3f,%.3f m"%tuple(tr.position)
            if tr.velocity_valid:
                label+=" %.2f m/s"%np.linalg.norm(tr.velocity)
            cv2.putText(top,label,tuple(center+np.array([9,-9])),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        cv2.putText(top,self.last_state.status+" | goals entered Y:%d B:%d"%(self.rules.scores["yellow"],self.rules.scores["blue"]),(12,24),cv2.FONT_HERSHEY_SIMPLEX,.60,(255,255,255),2)
        if self.params["publish_image"]:
            msg=self.bridge.cv2_to_imgmsg(top,encoding="bgr8")
            msg.header.stamp=stamp
            msg.header.frame_id="referee_field"
            self.image_pub.publish(msg)
        if self.params["show_ui"]:
            cv2.imshow("Pinky referee | metric top view",top)
            cv2.waitKey(1)

    def watchdog(self):
        if self.last_arrival is None or self.stale:
            return
        if time.monotonic()-self.last_arrival>self.params["frame_timeout_s"]:
            self.stale=True
            self.ball.miss()
            for tr in self.robots:
                tr.miss()
            self.rules.previous=None
            self.rules.pending=None
            self.rules.count=0
            self.publish(self.get_clock().now().to_msg(),"VIDEO_STALE")

    def clear_calibration(self):
        self.field=None
        self.goals={}
        self.frames=0
        self.rule_enabled=False
        self.ball=Track(0)
        self.robots=[Track(1),Track(2)]
        self.rules.reset()

    def get_state(self,req,res):
        res.state=self.last_state
        return res

    def is_out(self,req,res):
        res.success=self.rules.last_event=="OUT"
        res.message=self.rules.last_event or "NO_EVENT"
        return res

    def is_goal(self,req,res):
        res.success=self.rules.last_event=="GOAL"
        res.message=self.rules.last_event or "NO_EVENT"
        return res

    def reset_play(self,req,res):
        self.rules.reset()
        self.publish(self.get_clock().now().to_msg(),"WAIT_FOR_BALL" if self.rule_enabled else "RULES_DISABLED")
        res.success=True
        res.message="Play reset; score preserved"
        return res

    def reset_score(self,req,res):
        self.rules.scores={"yellow":0,"blue":0}
        return self.reset_play(req,res)

    def recalibrate(self,req,res):
        self.clear_calibration()
        self.publish(self.get_clock().now().to_msg(),"WAIT_FOR_CALIBRATION")
        res.success=True
        res.message="Calibration cleared; will retry on incoming images"
        return res

    def close(self):
        if self.cap is not None:
            self.cap.release()
        if self.params["show_ui"]:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node=None
    try:
        node=Referee()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
