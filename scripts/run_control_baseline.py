#!/usr/bin/env python3
"""Isolated unchanged-controller baseline. Never runs against hardware.

fixed: sensor/plant-only launch, one scripted cmd_vel publisher.
gotopose: full sensor localization + existing BT GoToPose, all avoidance OFF.
Truth is subscribed by this recorder ONLY. No truth is passed into any goal or controller.
"""
import argparse
import atexit
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import psutil
import rclpy
from rclpy.parameter import Parameter,parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy,QoSProfile,qos_profile_sensor_data
from rclpy.serialization import serialize_message
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import JointState,LaserScan,Imu,CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from pinky_soccer_msgs.msg import WorldModel,BehaviorGoal,MotionResult

from benchmark_localization import LOCAL_DDS
from analyze_gotopose_bag import normalize
from control_baseline_metrics import metrics

COURSES=dict(straight=[-.1,0.,0.],short=[-.35,0.,0.],rotate=[-.4,0.,math.pi/2],
             half_turn=[-.4,0.,math.pi],curve=[-.1,.2,math.pi/2],reverse=[-.6,0.,0.],
             long=[.5,0.,0.])


def stop_process_group(process):
    """Also used by atexit if setup fails before the main recording loop."""
    if process.poll() is not None:return
    try:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait()
    except ProcessLookupError:pass


def main():
    root=Path(__file__).resolve().parents[1]
    source_hashes={str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__),root/'src/pinky_soccer_mpc/pinky_soccer_mpc/mpc_controller.py',
                     root/'src/pinky_soccer_mpc/pinky_soccer_mpc/timing_trace.py'] if path.exists()}
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sim',choices=['mujoco','gazebo'],required=True)
    ap.add_argument('--mode',choices=['fixed','gotopose'],required=True)
    ap.add_argument('--case',choices=[*COURSES,'basic'],default='straight')
    ap.add_argument('--domain',type=int,default=225)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--initial-yaw',type=float,default=0.)
    ap.add_argument('--timeout',type=float,default=18.)
    ap.add_argument('--bag',action='store_true',help='Also save selected full CDR sensor/control topics; excludes RGB images')
    ap.add_argument('--trace',action='store_true',help='Enable diagnostic-only MPC slot and canceled-job JSON fields')
    ap.add_argument('--completion-mode',choices=['DEFAULT','LEGACY','STRICT_STOP','PRACTICAL_STOP'],default='DEFAULT')
    args=ap.parse_args()
    if (not 1<=args.domain<=232 or not math.isfinite(args.timeout) or args.timeout<=0
            or not math.isfinite(args.initial_yaw)):ap.error('Invalid isolated domain/timeout/yaw')
    if args.bag:
        import rosbag2_py  # Fail before launching any child if recording is unavailable.
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),CYCLONEDDS_URI=LOCAL_DDS,
                      GZ_PARTITION='pinky_control_'+uuid.uuid4().hex,ROS_LOG_DIR=str(args.output/'ros_logs'))
    os.environ['AMENT_PREFIX_PATH']=':'.join(p for p in os.environ.get('AMENT_PREFIX_PATH','').split(':')
                                           if not p.endswith('/install/pinky_soccer_skills'))
    if args.mode=='gotopose':
        launch=['ros2','launch','pinky_soccer_viz','game.launch.py',f'mode:={args.sim}',
            'headless:=true','verify:=true','preparation:=false','field:=hardboard',
            'initial_x:=-0.4','initial_y:=0',f'initial_yaw:={args.initial_yaw}',
            'ball_x:=0.7','ball_y:=0.45',f'heading_record:={args.output.resolve()}/records/heading.yaml']
    else:
        launch=['ros2','launch','pinky_soccer_sim',args.sim+'_soccer.launch.py','field:=hardboard',
            'x:=-0.4','y:=0',f'yaw:={args.initial_yaw}','spawn_ball:=false','imu_enabled:=true',
            'viewer:=false' if args.sim=='mujoco' else 'gui:=false']
    log=(args.output/'launch.log').open('w')
    processes=[subprocess.Popen(launch,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)]
    atexit.register(stop_process_group,processes[0])
    if args.mode=='fixed' and args.sim=='gazebo':
        processes.append(subprocess.Popen(['ros2','run','pinky_soccer_viz','gazebo_truth_relay',
            '--ros-args','-p','world_name:=hardboard_soccer'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        atexit.register(stop_process_group,processes[-1])
    rclpy.init()
    node=rclpy.create_node('control_baseline_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
    rows=[];episodes=[];cpu=[];latest={};topology=[];ready=False;calibrated=False
    start=time.monotonic()
    recorder=gzip.open(args.output/'samples.jsonl.gz','wt')
    bag=None
    if args.bag:
        bag=rosbag2_py.SequentialWriter()
        bag.open(rosbag2_py.StorageOptions(uri=str(args.output/'bag'),storage_id='sqlite3'),
                 rosbag2_py.ConverterOptions('cdr','cdr'))
    definitions=[('cmd','cmd_vel',Twist),('mpc_command','localization/controller_cmd_vel',Twist),
        ('drive','drive/cmd_vel',Twist),('plant_command','drive/motor_cmd_vel',Twist),
        ('joints','joint_states',JointState),('wheel_joints','wheel_joint_states',JointState),
        ('raw_joints','wheel_joint_states_raw',JointState),('odom','odom',Odometry),
        ('localization','localization/pose',PoseWithCovarianceStamped),
        ('world','world_model',WorldModel),('status','mpc/status',String),
        ('bt','behavior_test/status',String),('localization_status','localization/status',String),
        ('preparation','preparation/status',String),('bridge','sim/bridge_diag',String),
        ('goal','behavior_goal',BehaviorGoal),('result','motion/result',MotionResult),
        ('truth','ground_truth',PoseStamped),('tf','tf',TFMessage),('scan','scan',LaserScan),
        ('imu','imu_raw',Imu),('camera_info','camera/camera_info',CameraInfo),
        ('path','mpc/predicted_path',NavPath)]
    def callback(role,topic,msg):
        nonlocal ready
        ros_ns=node.get_clock().now().nanoseconds
        if bag is not None:bag.write('/'+topic,serialize_message(msg),ros_ns)
        wall=time.monotonic()-start
        try:
            alias={'mpc_command':'cmd','plant_command':'cmd','localization':'amcl',
                   'localization_status':'status','preparation':'status','bt':'status','bridge':'status','motion':'status'}.get(role,role)
            if role=='truth':
                q=msg.pose.orientation
                d=dict(x=msg.pose.position.x,y=msg.pose.position.y,
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            elif role=='path':
                d=dict(points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])
            elif role=='scan':d=dict(valid_points=int(np.isfinite(msg.ranges).sum()))
            elif role=='imu':d=dict(w=msg.angular_velocity.z)
            elif role=='camera_info':d=dict(width=msg.width,height=msg.height,k=list(msg.k))
            else:
                joints=('left_wheel_joint','right_wheel_joint') if role=='raw_joints' else ('l_wheel_joint','r_wheel_joint')
                d=normalize(alias,msg,*joints)
                if d is None:return
            if hasattr(msg,'header'):
                d['stamp_ns']=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            row=dict(d,role=role,topic='/'+topic,bag_ns=ros_ns,wall=wall)
            rows.append(row);latest[role]=row
            recorder.write(json.dumps(row,allow_nan=True)+'\n')
            if role=='localization_status':ready=bool(d['data'].get('ready'))
        except (ValueError,TypeError,AttributeError) as e:
            recorder.write(json.dumps(dict(role='decode_error',topic=topic,error=str(e),wall=wall,bag_ns=ros_ns))+'\n')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for role,topic,msg in definitions:
        if bag is not None:
            type_name=msg.__module__.split('.')[0]+'/msg/'+msg.__name__
            bag.create_topic(rosbag2_py.TopicMetadata(id=0,name='/'+topic,type=type_name,serialization_format='cdr'))
        node.create_subscription(msg,topic,lambda m,r=role,t=topic:callback(r,t,m),
            latched if role in ('localization_status','preparation') else qos_profile_sensor_data)
    request=node.create_publisher(String,'behavior_test/request',10)
    command=node.create_publisher(Twist,'cmd_vel',10) if args.mode=='fixed' else None
    calibrate=node.create_client(Trigger,'localization/calibrate')
    parameter_client=AsyncParameterClient(node,'mpc_controller') if args.mode=='gotopose' else None
    parameter_future=None;trace_future=None;runtime_parameters={}
    parameter_names=['target','calibration_profile','identification','mpc_backend','rate_hz','command_rate_hz',
        'horizon','steps','time_optimal_max_steps','time_optimal_budget_sec','max_lin','max_ang',
        'wheel_feedback','mpc_observer_mode','mpc_damping_scale','pos_tol','ang_tol','settle_sec',
        'goal_timeout','pose_timeout','init_timeout_sec','avoid_margin','avoid_solver_margin','timing_trace',
        'motion_management','motion_completion','motion_collision_monitor','motion_pose_tau','motion_latency','motion_deceleration']
    state='startup';phase=0;phase_start=None;episode=None;pause_until=None;last_cpu=last_send=last_ping=last_live=0.
    course=[k for k in COURSES if k!='long'] if args.case=='basic' else [args.case]
    def send_request(payload):request.publish(String(data=json.dumps(payload)))
    try:
        while time.monotonic()-start<180:
            rclpy.spin_once(node,timeout_sec=.002)
            elapsed=time.monotonic()-start
            if processes[0].poll() is not None:
                episodes.append(dict(case=episode['case'] if episode else 'startup',
                    start_wall=episode['start_wall'] if episode else 0,end_wall=elapsed,result='launch_exited'))
                break
            if args.trace and parameter_client is not None and trace_future is None and parameter_client.services_are_ready():
                trace_future=parameter_client.set_parameters([Parameter('timing_trace',value=True)])
            if trace_future is not None and trace_future.done() and not all(r.successful for r in trace_future.result().results):
                raise RuntimeError('Diagnostic timing_trace parameter was rejected')
            trace_ready=not args.trace or (trace_future is not None and trace_future.done())
            if trace_ready and parameter_client is not None and parameter_future is None and parameter_client.services_are_ready():
                parameter_future=parameter_client.get_parameters(parameter_names)
            if parameter_future is not None and parameter_future.done() and not runtime_parameters:
                runtime_parameters=dict(zip(parameter_names,[parameter_value_to_python(v)
                    for v in parameter_future.result().values]))
                (args.output/'runtime_parameters.json').write_text(json.dumps(runtime_parameters,indent=2))
            if elapsed-last_cpu>1:
                last_cpu=elapsed
                topology.append(dict(wall=elapsed,publishers={t:[p.node_name for p in node.get_publishers_info_by_topic(t)]
                    for t in ('/cmd_vel','/drive/cmd_vel','/localization/controller_cmd_vel')}))
                for child in [psutil.Process(),*psutil.Process(processes[0].pid).children(recursive=True)]:
                    try:cpu.append(dict(wall=elapsed,pid=child.pid,name=Path(child.cmdline()[1] if len(child.cmdline())>1 else child.name()).name,
                                         cpu_s=sum(child.cpu_times()[:2]),rss=child.memory_info().rss))
                    except psutil.Error:pass
            if elapsed-last_ping>.4:
                last_ping=elapsed;send_request(dict(command='ping'))
            if state=='startup':
                if elapsed>35:
                    episodes.append(dict(case='startup',start_wall=0,end_wall=elapsed,result='not_ready'));break
                if args.mode=='fixed':
                    if elapsed>5 and 'odom' in latest and 'joints' in latest:
                        if len(node.get_publishers_info_by_topic('/cmd_vel'))!=1:
                            raise RuntimeError('fixed-input test requires exactly one cmd_vel publisher')
                        state='run';phase_start=elapsed
                        episode=dict(case='fixed_profile',start_wall=elapsed)
                else:
                    if elapsed>5 and not calibrated and 'odom' in latest and calibrate.service_is_ready():
                        calibrate.call_async(Trigger.Request());calibrated=True
                    if (ready and 'world' in latest and latest['world'].get('confidence',0)>0
                            and runtime_parameters and 'status' in latest and trace_ready):
                        state='run';phase_start=elapsed
            if state=='run':
                if args.mode=='fixed':
                    t=elapsed-phase_start
                    # Fixed profile: straight, stop, rotate, stop, triangular ramp.
                    v,w=(.06,0.) if t<3 else (0.,0.) if t<4 else (0.,.4) if t<7 else (0.,0.) if t<8 else (
                        .04*min(t-8,12-t),0.) if t<12 else (0.,0.)
                    if elapsed-last_send>=.019:
                        last_send=elapsed;cmd=Twist();cmd.linear.x,cmd.angular.z=float(v),float(w);command.publish(cmd)
                    if t>13:
                        episode.update(end_wall=elapsed,result='profile_completed');episodes.append(episode);break
                else:
                    if episode is None:
                        name=course[phase];goal=COURSES[name]
                        episode=dict(case=name,goal=goal,start_wall=elapsed,
                                     start_estimate=latest.get('world'),request_id=15000+phase)
                        send_request(dict(command='run',id=episode['request_id'],target='GoToPose',
                            ports={**dict(zip(('x','y','theta'),map(str,goal))),
                                   'avoid_walls':'false','avoid_opponents':'false','avoid_ball':'false',
                                   'completion_mode':args.completion_mode},
                            timeout_sec=args.timeout))
                    status=latest.get('bt',{}).get('data',{})
                    terminal=status.get('request_id')==episode['request_id'] and status.get('state') in (
                        'success','failure','timeout','error','world_model_stale','stopped')
                    if terminal or elapsed-episode['start_wall']>args.timeout+2:
                        episode.update(end_wall=elapsed,result=status.get('state') if terminal else 'no_terminal_response')
                        episodes.append(episode);episode=None;phase+=1
                        send_request(dict(command='stop'))
                        if phase>=len(course):break
                        state='pause';pause_until=elapsed+1
            elif state=='pause' and elapsed>=pause_until:state='run'
            if elapsed-last_live>=.5:
                last_live=elapsed
                (args.output/'live.json').write_text(json.dumps(dict(wall=elapsed,state=state,phase=phase,
                    localization=latest.get('localization_status',{}).get('data',{}).get('state'),
                    mpc=latest.get('status',{}).get('data',{}).get('reason'))))
    finally:
        if command is not None:command.publish(Twist())
        else:send_request(dict(command='stop'))
        for process in processes:
            stop_process_group(process)
        recorder.close();bag=None
        node.destroy_node();rclpy.try_shutdown();log.close()
    summary=[metrics(rows,e) for e in episodes]
    (args.output/'episodes.json').write_text(json.dumps(summary,indent=2))
    (args.output/'metadata.json').write_text(json.dumps(dict(arguments={**vars(args),'output':str(args.output)},
        launch=launch,avoidance=False,completion=args.completion_mode,truth_use='evaluation only',
        source_sha256_at_start=source_hashes,
        recorder_note='Normalized gzip plus optional full CDR recording add overhead; recorder CPU is included. Live file limited to 2 Hz.',
        note='Sequential basic course records actual initial states; it does not assert identical starts between legs.',
        startup_barrier='MPC parameter response and first status, plus sensor localization READY, before measured goal.',
        cpu=cpu,topology=topology,runtime_parameters=runtime_parameters),indent=2))
    print(json.dumps([{k:v for k,v in s.items() if k in ('case','result','elapsed_wall_s','truth_displacement_m',
        'truth_final_position_error_m','truth_final_heading_error_rad','zero_run_starts','solver_status_counts')}
        for s in summary],indent=2))
    return 0 if summary and all(s['result'] in ('success','profile_completed') for s in summary) else 2


if __name__=='__main__':
    raise SystemExit(main())
