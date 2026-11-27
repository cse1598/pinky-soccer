#!/usr/bin/env python3
"""Own at most one official hardware LED server. Run on the robot, never the PC."""
import fcntl
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('soccer_led_supervisor')
    name = node.get_namespace().strip('/').replace('/', '_') or 'root'
    lock = open('/tmp/pinky_soccer_led_'+name+'.lock', 'w')
    child = None
    try:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            node.get_logger().info('Another soccer LED supervisor already owns this namespace')
            return
        deadline = time.monotonic()+2.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        service = node.get_namespace().rstrip('/')+'/set_led'
        existing = any(k in (service, service[:-7]+'set_lamp')
                       for k, _ in node.get_service_names_and_types())
        if existing:
            node.get_logger().info('Reusing existing '+service+' server; no duplicate started')
        else:
            child = subprocess.Popen(['ros2', 'run', 'pinky_led', 'led_server', '--ros-args',
                '-r', '__ns:='+node.get_namespace()], start_new_session=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
            if child is not None and child.poll() is not None:
                node.get_logger().error('Official LED server exited; inspect pinkylib on the robot')
                break
    except KeyboardInterrupt:
        pass
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
        lock.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
