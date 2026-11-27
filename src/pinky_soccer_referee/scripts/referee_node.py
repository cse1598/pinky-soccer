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
from referee_core import (Field, Track, RobotTrack, Rules, detect, orient_goals,
                          associate, robot_observations, associate_robots, trapezoid_points)


class Referee(Node):
    def __init__(self):
        super().__init__("referee", namespace="referee")
        defaults = {
            "image_topic": "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "video_path": "", "show_ui": False, "publish_image": True,
            "field_length_m": 1.670, "field_width_m": 1.070,
            "pixels_per_meter": 550., "margin_m": .15,
            "ball_radius_m": .02135, "warmup_frames": 30,
            "require_camera_info": False, "allow_approximate_rules": False,
            "yellow_goal_y_min": 0., "yellow_goal_y_max": 0.,
            "blue_goal_y_min": 0., "blue_goal_y_max": 0.,
            "yellow_goal_side": -1, "blue_goal_side": 1,
            "frame_timeout_s": .5,
            "robot_match_gate_m": .12, "robot_reacquire_frames": 2,
            "blue_led_min_light": 125, "velocity_arrow_min_mps": .035,
            "distortion_mode": "auto", "calibration_frames": 9,
            "calibration_max_error_px": 5.0, "debug_ui": False,
        }
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.params = {k:self.get_parameter(k).value for k in defaults}
        p=self.params
        if min(p["field_length_m"],p["field_width_m"],p["pixels_per_meter"],p["ball_radius_m"],p["margin_m"],p["frame_timeout_s"]) <= 0:
            raise ValueError("Set positive measured white-line field_length_m and field_width_m; see README")
        if p["yellow_goal_side"] != -1 or p["blue_goal_side"] != 1:
            raise ValueError("Canonical coordinates require yellow=-1, blue=+1")
        if p["robot_match_gate_m"]<=0 or p["robot_reacquire_frames"]<1 or not 0<=p["blue_led_min_light"]<=255 or p["velocity_arrow_min_mps"]<0:
            raise ValueError("Invalid robot tracking/LED parameters")
        if p['distortion_mode'] not in ('auto','camera_info','plumb_line','none') or not 3<=p['calibration_frames']<=30 or p['calibration_max_error_px']<=0:
            raise ValueError('Invalid calibration settings')
        self.bridge=CvBridge()
        self.field=None
        self.camera=None
        self.goals={}
        self.rules=Rules()
        self.ball=Track(0)
        self.robots=[RobotTrack(1),RobotTrack(2)]
        self.frames=0
        self.calibration_buffer=[]
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
        field=Field(image,p["field_length_m"],p["field_width_m"],p["pixels_per_meter"],p["margin_m"],camera,
                    p["distortion_mode"],p["calibration_max_error_px"])
        goals=orient_goals(field,image)
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
        self.get_logger().info("Calibrated: "+field.source+"; tape p90 px="+str(np.round(field.boundary_errors,2))+"; goal intervals="+str(goals)+"; rules enabled="+str(self.rule_enabled))

    @staticmethod
    def msg(tr):
        m=TrackedObject()
        m.id=tr.id
        m.visible=tr.visible
        m.velocity_valid=tr.velocity_valid
        m.heading_valid=bool(tr.visible and getattr(tr,"heading_measured",False))
        m.heading=float(tr.heading) if m.heading_valid else float("nan")
        m.angular_velocity_valid=bool(m.heading_valid and getattr(tr,"angular_velocity",None) is not None)
        m.angular_velocity=float(tr.angular_velocity) if m.angular_velocity_valid else float("nan")
        m.team=getattr(tr,"team","unknown") if tr.visible else "unknown"
        m.team_valid=bool(tr.visible and m.team in ("yellow","blue"))
        m.heading_confidence=float(getattr(tr,"heading_confidence",0.)) if m.heading_valid else 0.
        m.tracking_confidence=float(getattr(tr,"match_confidence",0.)) if tr.visible else 0.
        m.reacquired=bool(tr.visible and getattr(tr,"reacquired",False))
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
            if self.frames % 3 != 0:
                return
            if self.calibration_buffer and self.calibration_buffer[0].shape!=image.shape:
                self.calibration_buffer.clear()
            self.calibration_buffer.append(image.copy())
            if len(self.calibration_buffer)<self.params["calibration_frames"]:
                return
            # Median of spatially aligned fixed-camera frames suppresses moving
            # robots and exposure fluctuations during static map initialization.
            calibration_image=np.median(np.stack(self.calibration_buffer),axis=0).astype(np.uint8)
            self.calibration_buffer.clear()
            try:
                self.calibrate(calibration_image)
            except Exception as exc:
                self.get_logger().warning("Calibration failed: "+str(exc),throttle_duration_sec=2.)
                self.publish(stamp,"WAIT_FOR_CALIBRATION")
                if self.params["show_ui"]:
                    preview=image.copy()
                    cv2.putText(preview,"CALIBRATION RETRY - see terminal",(12,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,230,255),2)
                    cv2.imshow("Pinky referee | metric top view",preview)
                    cv2.waitKey(1)
                return
        f=self.field
        top=f.warp(image)
        bs,rs=detect(top,f,self.params["ball_radius_m"])
        associate([self.ball],[f.metric(v[0]) for v in bs[:4]],t,gate=.45)
        observations=robot_observations(top,f,rs[:5],self.params["blue_led_min_light"],self.robots)
        associate_robots(self.robots,observations,t,self.params["robot_match_gate_m"],self.params["robot_reacquire_frames"])
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
        cv2.rectangle(top,tuple(f.pixel((-l,w))),tuple(f.pixel((l,-w))),(255,255,0),2,cv2.LINE_AA)
        origin=f.pixel((0.,0.))
        cv2.drawMarker(top,tuple(origin),(255,255,255),cv2.MARKER_CROSS,16,2,cv2.LINE_AA)
        cv2.putText(top,"(0, 0)",tuple(origin+np.array([9,17])),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
        for side,(goal_color,lo,hi) in self.goals.items():
            # Match each goal-line segment to its goal color.
            label_color=(0,230,255) if goal_color=="yellow" else (255,140,0)
            cv2.line(top,tuple(f.pixel((side*l,lo))),tuple(f.pixel((side*l,hi))),label_color,4,cv2.LINE_AA)
            label_pos=f.pixel((side*l,(lo+hi)/2))
            offset=np.array([8,-12]) if side<0 else np.array([-90,-12])
            cv2.putText(top,goal_color.upper()+" GOAL",tuple(label_pos+offset),cv2.FONT_HERSHEY_SIMPLEX,.45,label_color,1,cv2.LINE_AA)
        for tr in [self.ball]+self.robots:
            if not tr.visible:
                continue
            center=f.pixel(tr.position)
            if tr.id==0:
                color=(0,130,255)
                cv2.circle(top,tuple(center),max(3,round(self.params["ball_radius_m"]*f.ppm)),color,2,cv2.LINE_AA)
                label="Ball"
            else:
                color={"yellow":(0,230,255),"blue":(255,140,0),"unknown":(0,255,70)}[tr.team]
                if tr.heading is not None:
                    cv2.polylines(top,[trapezoid_points(tr,f)],True,color,2,cv2.LINE_AA)
                elif tr.bbox is not None:
                    x,y,bw,bh=np.rint(tr.bbox*f.ppm).astype(int)
                    cv2.rectangle(top,(x,y),(x+bw,y+bh),color,1,cv2.LINE_AA)
                label="Pinky %d | %s"%(tr.id,tr.team.upper())
                if tr.heading is not None:
                    label+=" %.1f deg"%math.degrees(tr.heading)
                    if not tr.heading_measured:
                        label+=" (PRED)"
                else:
                    label+=" | yaw ?"
                if tr.reacquired:
                    label+=" | REACQUIRED"
            cv2.drawMarker(top,tuple(center),color,cv2.MARKER_CROSS,10,2,cv2.LINE_AA)
            speed=float(np.linalg.norm(tr.velocity)) if tr.velocity_valid else 0.
            if tr.velocity_valid and speed>self.params["velocity_arrow_min_mps"]:
                cv2.arrowedLine(top,tuple(center),tuple(f.pixel(tr.position+.30*tr.velocity)),color,2,cv2.LINE_AA,tipLength=.23)
            label+=" %.3f,%.3f m"%tuple(tr.position)
            if tr.velocity_valid:
                label+=" %.2f m/s"%speed
            (tw,th),_=cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,.43,1)
            text_pos=(int(np.clip(center[0]+9,2,max(2,top.shape[1]-tw-3))),int(np.clip(center[1]-max(22,int(.09*f.ppm)),th+3,top.shape[0]-3)))
            cv2.putText(top,label,text_pos,cv2.FONT_HERSHEY_SIMPLEX,.43,(15,15,15),3,cv2.LINE_AA)
            cv2.putText(top,label,text_pos,cv2.FONT_HERSHEY_SIMPLEX,.43,color,1,cv2.LINE_AA)
        cv2.putText(top,self.last_state.status+" | goals entered Y:%d B:%d"%(self.rules.scores["yellow"],self.rules.scores["blue"]),(12,24),cv2.FONT_HERSHEY_SIMPLEX,.60,(255,255,255),2)
        if self.params["debug_ui"]:
            debug="CAL %s | tape p90 %s px"%(f.source,"/".join("%.1f"%x for x in f.boundary_errors))
            cv2.putText(top,debug,(12,45),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
            for i,tr in enumerate(self.robots):
                status="MEASURED" if tr.heading_measured else ("PRED" if tr.heading is not None else "MISSING")
                debug="R%d LED candidates=%d yaw=%s conf=%.2f track=%.2f"%(tr.id,tr.led_candidates,status,tr.heading_confidence,tr.match_confidence)
                cv2.putText(top,debug,(12,65+20*i),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
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
        self.calibration_buffer=[]
        self.rule_enabled=False
        self.ball=Track(0)
        self.robots=[RobotTrack(1),RobotTrack(2)]
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
