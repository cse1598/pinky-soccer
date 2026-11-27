#!/usr/bin/env python3
"""One isolated MuJoCo BT sequence: prepared successor acquisition, no truth feedback."""
import argparse
import gzip
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist,PoseStamped
from pinky_soccer_msgs.msg import BehaviorGoal,MotionResult
from benchmark_localization import LOCAL_DDS
from run_control_baseline import stop_process_group
from analyze_gotopose_bag import normalize


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['PASS_THROUGH','HANDOFF'],required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);out=args.output.resolve()
    os.environ.update(ROS_DOMAIN_ID='226',CYCLONEDDS_URI=LOCAL_DDS,
        GZ_PARTITION='part_b_'+uuid.uuid4().hex,ROS_LOG_DIR=str(out/'logs'),OPENBLAS_NUM_THREADS='1')
    root=Path(__file__).resolve().parents[1]
    profile=root/'src/pinky_soccer_sim/config/calibration/mujoco_profile.yaml'
    tree=out/'sequence.xml'
    tree.write_text(f'''<root BTCPP_format="4"><BehaviorTree ID="Main"><Sequence>
    <ReadWorldModel topic_name="world_model" pose="{{@pose}}" ball="{{@ball}}" playing="{{@playing}}" opponents="{{@opponents}}" own_goal="{{@own_goal}}" opp_goal="{{@opp_goal}}"/>
    <GoToPose x="-0.2" y="0" theta="0" completion_mode="{args.mode}" next_ready="true" next_x="0.1" next_y="0" next_theta="0"/>
    <GoToPose x="0.1" y="0" theta="0" completion_mode="STRICT_STOP"/><StopRobot/>
    </Sequence></BehaviorTree><BehaviorTree ID="PublishGoal"><PublishBehaviorGoal topic_name="behavior_goal"/></BehaviorTree></root>''')
    launch=out/'transition.launch.py'
    launch.write_text(f'''from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pinky_soccer_viz.robot_stack import RobotConfig, decision_stack
from pinky_soccer_sim.field_selection import field_files

def generate_launch_description():
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('pinky_soccer_sim')+'/launch/mujoco_soccer.launch.py'),launch_arguments={{'viewer':'false','field':'hardboard','x':'-0.4','spawn_ball':'false','imu_enabled':'true','calibration_profile':{str(profile)!r}}}.items())
    cfg=RobotConfig(preparation=False,ui=False,groot2=False,simulation=True,target='mujoco',field_spec=str(field_files('hardboard')[1]),initial=(-.4,0.,0.),calibration_profile={str(profile)!r},bt_tree_file={str(tree)!r},heading_record={str(out/'heading.yaml')!r},max_ang=2.)
    return LaunchDescription([sim,*decision_stack(cfg)])
''')
    log=(out/'launch.log').open('w');process=subprocess.Popen(['ros2','launch',str(launch)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('part_b_transition_verifier');start=time.monotonic()
    rows=[];goals={};successes={};latest={};calibrated=False
    stream=gzip.open(out/'samples.jsonl.gz','wt')
    def cb(role,msg):
        if role=='truth':
            d=dict(x=msg.pose.position.x,y=msg.pose.position.y)
        else:d=normalize('status' if role=='status' else role,msg,'l_wheel_joint','r_wheel_joint')
        row=dict(d,role=role,wall=time.monotonic()-start);rows.append(row);latest[role]=row
        stream.write(json.dumps(row)+'\n')
        if role=='goal' and row['action']=='GoToPose':goals[row['goal_id']]=row
        if role=='result' and row['state']==MotionResult.SUCCESS:successes.setdefault(row['goal_id'],row)
    for role,topic,typ in [('cmd','cmd_vel',Twist),('status','mpc/status',String),('goal','behavior_goal',BehaviorGoal),
                            ('result','motion/result',MotionResult),('truth','ground_truth',PoseStamped)]:
        node.create_subscription(typ,topic,lambda m,r=role:cb(r,m),qos_profile_sensor_data)
    client=node.create_client(Trigger,'localization/calibrate')
    try:
        while time.monotonic()-start<30 and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.002)
            if not calibrated and time.monotonic()-start>5 and client.service_is_ready():
                client.call_async(Trigger.Request());calibrated=True
            if any(gid in successes and abs(g['x']-.1)<1e-6 for gid,g in goals.items()):break
    finally:
        stop_process_group(process);stream.close();node.destroy_node();rclpy.try_shutdown();log.close()
    first=[s for gid,s in successes.items() if gid in goals and abs(goals[gid]['x']+.2)<1e-6]
    second=[s for gid,s in successes.items() if gid in goals and abs(goals[gid]['x']-.1)<1e-6]
    transition=[]
    if first:
        t=first[0]['wall'];transition=[r for r in rows if r['role']=='cmd' and t-.04<=r['wall']<=t+.10]
    result=dict(mode=args.mode,first_success=bool(first),second_success=bool(second),
        first_reason=first[0]['reason'] if first else None,
        transition_command_samples=len(transition),
        transition_zero_samples=sum(abs(r['v'])+abs(r['w'])<1e-6 for r in transition),
        goals=list(goals.values()),final_status=latest.get('status'),final_truth=latest.get('truth'))
    (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if first and second and transition and not result['transition_zero_samples'] else 2


if __name__=='__main__':raise SystemExit(main())
