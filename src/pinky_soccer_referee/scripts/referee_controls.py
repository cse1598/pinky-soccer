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
                   and s.get('localization_valid') is True
                   and s.get('referee_start_allowed', True) and not s.get('start_pending', False)
                   for s, _ in statuses)
    if action in ('prepare', 'retry'):
        return all(s.get('state') in ('IDLE', 'ERROR', 'STOPPED', 'READY', 'RECOVERING') for s, _ in statuses)
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
        self.auto_names = self.names == ['']
        self.status = {}
        self.pending = {}
        self.message = 'Waiting for robot preparation/status'
        self.feedback = {}
        self.buttons = []
        self.stop_requested = False
        self.last_stop = -float('inf')
        self.request_type = Trigger.Request
        self.action_clients = {}
        self.subscriptions = []
        for name in self.names:
            self.connect(name)
        self.timer = node.create_timer(.2, self.poll)

    def connect(self, name):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        prefix = ('/'+name if name else '')+'/preparation/'
        self.action_clients[name] = {a: self.node.create_client(Trigger, prefix+a) for a in self.ACTIONS}
        self.subscriptions.append(self.node.create_subscription(
            String, prefix+'status', lambda msg, name=name: self.receive(name, msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def discover_names(self):
        if not self.auto_names:
            return
        if self.action_clients['']['prepare'].service_is_ready():
            return  # Existing single robot without a namespace.
        services = {name for name, _ in self.node.get_service_names_and_types()}
        found = [name for name in ('yellow', 'blue') if '/'+name+'/preparation/prepare' in services]
        if not found:
            return
        for name in ('yellow', 'blue'):
            if name not in self.action_clients:
                self.connect(name)
        # Auto mode is the canonical two-team match. A named single robot can
        # explicitly set robot_namespaces:=yellow (or blue).
        self.names = ['yellow', 'blue']
        self.auto_names = False

    def receive(self, name, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                old_phase = self.status.get(name, ({}, 0.))[0].get('state')
                self.status[name] = status, time.monotonic()
                if status.get('state') != old_phase:
                    self.feedback.pop(name, None)
                if (self.message == 'Waiting for robot preparation/status'
                        and all(n in self.status for n in self.names)):
                    self.message = 'Robot connected: PREPARE -> READY -> START'
        except (ValueError, TypeError):
            pass

    def enabled(self, action):
        if action in ('prepare', 'retry'):
            return True  # The click explains unavailable/busy robots instead of going dead.
        states = [self.status.get(n, ({}, -float('inf'))) for n in self.names]
        return (allowed(action, states, time.monotonic(), bool(self.pending))
                and all(self.action_clients[n][action].service_is_ready() for n in self.names))

    def request(self, action):
        if action not in self.ACTIONS:
            return
        if action in ('prepare', 'retry'):
            self.discover_names()
            if self.pending:
                self.message = 'Request pending: '+', '.join((n or 'root')+'/'+a for n,a in self.pending)
                return
            now = time.monotonic()
            if any(self.status.get(n, ({}, -float('inf')))[0].get('state') == 'PLAYING'
                   and now-self.status[n][1] <= 1.5 for n in self.names):
                self.message = 'Game is playing: STOP [S] before preparing again'
                return
            messages = []
            for name in self.names:
                label = name or 'root'
                status, seen = self.status.get(name, ({}, -float('inf')))
                client = self.action_clients[name][action]
                if not client.service_is_ready():
                    messages.append(label+': waiting for /'+name.strip('/')+('/' if name else '')+'preparation/'+action)
                elif now-seen > 1.5:
                    messages.append(label+': preparation/status not received or stale')
                elif status.get('state') in ('CALIBRATING', 'MOVING_TO_START'):
                    messages.append(label+': preparation already running')
                elif action == 'prepare' and status.get('state') == 'READY':
                    messages.append(label+': already READY; use RETRY to prepare again')
                elif status.get('state') in ('IDLE', 'ERROR', 'STOPPED', 'READY', 'RECOVERING'):
                    try:
                        self.pending[(name, action)] = client.call_async(self.request_type()), now
                        self.stop_requested = False
                        messages.append(label+': '+action+' requested')
                    except Exception as exc:
                        messages.append(label+': '+str(exc))
                else:
                    messages.append(label+': cannot prepare from '+str(status.get('state')))
                self.feedback[name] = messages[-1].removeprefix(label+': ')
            self.message = ' | '.join(messages)
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
        self.discover_names()
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
                self.feedback[name] = response.message
                busy = action in ('prepare', 'retry') and 'already running' in response.message.lower()
                failed |= not response.success and action != 'stop' and not busy
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
                led.get('color', '?'), ' blink' if led.get('blink') else '',
                self.feedback.get(name) or s.get('reason', ''))
            self.text(output, line, height+58+24*i)
        self.text(output, self.message, output.shape[0]-8)
        return output

    @staticmethod
    def text(frame, line, y):
        line = line.encode('ascii', errors='replace').decode()[:max(30, frame.shape[1]//7)]
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (230, 230, 230), 1)
