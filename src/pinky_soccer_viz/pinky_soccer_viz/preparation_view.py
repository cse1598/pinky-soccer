"""Small match controls alongside the existing camera/localization/map windows."""
import json
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class PreparationView(Node):
    def __init__(self):
        super().__init__('preparation_view')
        self.status = {}
        self.received = -float('inf')
        self.message = ''
        self.clients = {k: self.create_client(Trigger, 'preparation/'+k)
                        for k in ('prepare', 'start', 'stop', 'retry')}
        self.team_client = self.create_client(SetBool, 'preparation/set_team')
        self.create_subscription(String, 'preparation/status', self.receive,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.received = time.monotonic()
        except ValueError:
            pass

    def call(self, action, team=None):
        client = self.team_client if team else self.clients[action]
        if not client.service_is_ready():
            self.message = '상태 관리자 연결 대기'
            return
        future = client.call_async(SetBool.Request(data=team == 'blue') if team else Trigger.Request())
        def done(f):
            try:
                result = f.result()
                self.message = result.message
            except Exception as e:
                self.message = str(e)
        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = PreparationView()
    root = tk.Tk()
    root.title('Pinky 경기 준비')
    panel = ttk.Frame(root, padding=14)
    panel.pack(fill='both', expand=True)
    state = tk.StringVar(value='IDLE / 연결 대기')
    text = tk.StringVar()
    ttk.Label(panel, textvariable=state, font=('TkDefaultFont', 15, 'bold')).pack(anchor='w')
    team = ttk.Combobox(panel, values=('yellow', 'blue'), state='readonly', width=14)
    team.pack(anchor='w', pady=8)
    team.bind('<<ComboboxSelected>>', lambda event: node.call('', team=team.get()))
    row = ttk.Frame(panel)
    row.pack(fill='x')
    buttons = {}
    for label, key in [('경기 준비', 'prepare'), ('경기 시작', 'start'),
                       ('경기 중지', 'stop'), ('준비 재시도', 'retry')]:
        buttons[key] = ttk.Button(row, text=label, command=lambda k=key: node.call(k))
        buttons[key].pack(side='left', padx=3)
    ttk.Label(panel, textvariable=text, justify='left', wraplength=650).pack(anchor='w', pady=10)
    root.bind('<Escape>', lambda e: node.call('stop'))
    def update():
        if not rclpy.ok():
            root.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0.)
        s = node.status
        phase = s.get('state', 'CONNECTING')
        if time.monotonic()-node.received > 1.5:
            phase = 'CONNECTING' if not s else 'STATUS_STALE'
        state.set(phase+' / '+s.get('team', ''))
        if s.get('team'):
            team.set(s['team'])
        team.configure(state='readonly' if phase in ('IDLE', 'ERROR', 'STOPPED') else 'disabled')
        buttons['start'].configure(state='normal' if s.get('ready') and phase in ('READY', 'STOPPED')
                                  and s.get('localization_valid') else 'disabled')
        for k in ('prepare', 'retry'):
            buttons[k].configure(state='normal' if phase in ('IDLE', 'READY', 'STOPPED', 'ERROR')
                                 else 'disabled')
        loc = s.get('localization', {})
        led = s.get('led', {})
        text.set('\n'.join([f"센서: {s.get('sensors', {})} / odom fresh={s.get('odom_fresh')}",
            f"Localization: {loc.get('state', '대기')} / 색상={loc.get('selected_goal_color')}",
            f"자기 골대: {s.get('own_goal')} / 시작 Pose: {s.get('start_pose')}",
            f"이동 도착={s.get('arrived')} / 준비 완료={s.get('ready')}",
            f"LED: {led.get('color')} / 점멸={led.get('blink')} / 서비스={led.get('service')}",
            s.get('reason', ''), node.message]))
        root.after(80, update)
    root.after(0, update)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
