"""UI commands use the robot's preparation services, never velocity or vision poses."""
import json
import re
import time

import cv2
import numpy as np


def robot_namespaces(value):
    names = list(dict.fromkeys(part.strip().strip('/') for part in value.split(',')))
    if any(name and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', name)
           for name in names):
        raise ValueError('robot_namespaces must contain comma-separated ROS namespaces')
    return names


def allowed(action, statuses, now, pending=False):
    if action == 'stop':
        return True
    if pending or not statuses or any(now-seen > 1.5 for _, seen in statuses):
        return False
    if action == 'start':
        return all(s.get('state') in ('READY', 'STOPPED') and s.get('ready') is True
                   and s.get('localization_valid') is True for s, _ in statuses)
    if action in ('prepare', 'retry'):
        return all(s.get('state') in ('IDLE', 'ERROR', 'STOPPED', 'READY') for s, _ in statuses)
    return False


class MatchControls:
    ACTIONS = ('prepare', 'start', 'stop', 'retry')
    LABELS = ('PREPARE [P]', 'START [G]', 'STOP [S]', 'RETRY [R]')

    def __init__(self, node, namespaces):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.node = node
        self.names = robot_namespaces(namespaces)
        self.status = {}
        self.pending = {}
        self.message = 'Waiting for robot preparation/status'
        self.buttons = []
        self.stop_requested = False
        self.last_stop = -float('inf')
        self.request_type = Trigger.Request
        self.action_clients = {}
        self.subscriptions = []
        for name in self.names:
            prefix = ('/'+name if name else '')+'/preparation/'
            self.action_clients[name] = {a: node.create_client(Trigger, prefix+a) for a in self.ACTIONS}
            self.subscriptions.append(node.create_subscription(
                String, prefix+'status', lambda msg, name=name: self.receive(name, msg),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.timer = node.create_timer(.2, self.poll)

    def receive(self, name, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.status[name] = status, time.monotonic()
                if (self.message == 'Waiting for robot preparation/status'
                        and all(n in self.status for n in self.names)):
                    self.message = 'Robot connected: PREPARE -> READY -> START'
        except (ValueError, TypeError):
            pass

    def enabled(self, action):
        states = [self.status.get(n, ({}, -float('inf'))) for n in self.names]
        return (allowed(action, states, time.monotonic(), bool(self.pending))
                and all(self.action_clients[n][action].service_is_ready() for n in self.names))

    def request(self, action):
        if action not in self.ACTIONS:
            return
        if action == 'stop':
            self.stop_requested = True
            self.last_stop = time.monotonic()
            # Stop can interrupt an in-flight prepare/start; never wait on those replies.
            for future, _ in self.pending.values():
                future.cancel()
            self.pending.clear()
        elif not self.enabled(action):
            self.message = action+': blocked (waiting for fresh READY / service / previous request)'
            return
        else:
            self.stop_requested = False
        self.message = action+': requesting'
        for name in self.names:
            client = self.action_clients[name][action]
            if not client.service_is_ready():
                self.message = (name or 'root')+': '+action+' service unavailable'
                continue
            try:
                self.pending[(name, action)] = client.call_async(self.request_type()), time.monotonic()
            except Exception as exc:
                self.message = action+': '+str(exc)
                if action != 'stop':
                    self.request('stop')
                    return

    def poll(self):
        failed = False
        for key, (future, sent) in list(self.pending.items()):
            name, action = key
            if not future.done() and time.monotonic()-sent < 3.:
                continue
            self.pending.pop(key, None)
            try:
                if not future.done():
                    future.cancel()
                    raise TimeoutError('service response timeout')
                response = future.result()
                self.message = (name or 'root')+': '+response.message
                failed |= not response.success and action != 'stop'
            except Exception as exc:
                self.message = (name or 'root')+': '+str(exc)
                failed |= action != 'stop'
        if failed:
            reason = self.message
            self.request('stop')
            self.message = reason+'; requested STOP'
        elif (self.stop_requested and time.monotonic()-self.last_stop > .5
              and not any(action == 'stop' for _, action in self.pending)
              and any(s.get('state') in ('CALIBRATING', 'MOVING_TO_START', 'PLAYING')
                      and time.monotonic()-seen <= 1.5 for s, seen in self.status.values())):
            # Canceling a client future cannot cancel a request already at the server.
            # Honor STOP even if an earlier prepare/start response arrives late.
            self.request('stop')

    def mouse(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONUP:
            for action, (x0, y0, x1, y1) in self.buttons:
                if x0 <= x < x1 and y0 <= y < y1:
                    self.request(action)
                    break

    def key(self, key):
        action = {ord('p'): 'prepare', ord('g'): 'start', ord('s'): 'stop',
                  ord('r'): 'retry', 27: 'stop'}.get(key)
        if action:
            self.request(action)

    def render(self, frame):
        height, width = frame.shape[:2]
        footer = np.full((78+24*len(self.names), width, 3), 30, np.uint8)
        output = np.vstack((frame, footer))
        self.buttons = []
        step = max(1, (width-16)//4)
        for i, (action, label) in enumerate(zip(self.ACTIONS, self.LABELS)):
            box = (8+i*step, height+6, 8+(i+1)*step-5, height+38)
            active = self.enabled(action)
            color = (50, 50, 180) if action == 'stop' else (60, 110, 60)
            cv2.rectangle(output, box[:2], box[2:], color if active else (65, 65, 65), -1)
            cv2.putText(output, label, (box[0]+5, box[1]+22), cv2.FONT_HERSHEY_SIMPLEX,
                        .46, (255, 255, 255) if active else (140, 140, 140), 1)
            self.buttons.append((action, box))
        for i, name in enumerate(self.names):
            s, seen = self.status.get(name, ({}, -float('inf')))
            phase = s.get('state', 'WAITING') if time.monotonic()-seen <= 1.5 else 'STATUS STALE'
            led = s.get('led', {})
            line = '%s: %s | team=%s ready=%s | LED=%s%s | %s' % (
                name or 'root', phase, s.get('team', '?'), s.get('ready', False),
                led.get('color', '?'), ' blink' if led.get('blink') else '', s.get('reason', ''))
            self.text(output, line, height+58+24*i)
        self.text(output, self.message, output.shape[0]-8)
        return output

    @staticmethod
    def text(frame, line, y):
        line = line.encode('ascii', errors='replace').decode()[:max(30, frame.shape[1]//7)]
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (230, 230, 230), 1)
