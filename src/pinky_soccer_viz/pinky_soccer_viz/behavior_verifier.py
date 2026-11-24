"""
Behaviour verifier window: command single behaviours and watch how they run.

Left: the behaviours used by the game (catalog from pinky_soccer_bt/config/verify_catalog.yaml).
Middle: description, inputs (editable ports and read-only world inputs), run/stop, live BT node
states, live conditions, evaluation and run history.
Right: 2D map of what the robot believes (/world_model): own pose, ball, opponents, goals and
lines, the current BehaviorGoal target and planned_path, a controller path preview and the
driven trail. In simulation the ground truth can be overlaid (ball = red dot, robots = arrows).

The window talks to bt_node in verify_mode (see pinky_soccer_bt/verify_mode.hpp), so the same
C++ behaviour nodes and controller run in simulation and on the real robot. Ground-truth
topics are used only for display and evaluation, never as decision inputs.

Keys: Enter = run, Esc or Space = stop, F5 = repeat the last run.
Map: click = target position, drag = position + heading (Shift snaps to 15 deg),
double-click = set and run.
"""

import datetime
import json
import math
import os
import signal
import threading
import time
import tkinter as tk
from tkinter import ttk

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
from pinky_soccer_viz import style
from pinky_soccer_viz import verify_core as core
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

STATE_NAMES = {0: 'OBSERVED', 1: 'ESTIMATED', 2: 'LOST'}
STATUS_COLORS = {'RUNNING': '#f2c200', 'SUCCESS': '#3fb950', 'FAILURE': '#f85149',
                 'IDLE': '#8b949e', 'SKIPPED': '#8b949e'}
COLORS = {
    'bg': '#1e2227', 'panel': '#272c33', 'text': '#e6e6e6', 'muted': '#9aa4ae',
    'carpet': '#2e6b3a', 'wall': '#111111', 'line': '#f2f2f2', 'yellow': '#f2cc1a',
    'blue': '#2659e6', 'self': '#19ccf2', 'ball': '#ff7a00', 'opponent': '#e04040',
    'target': '#e61ae6', 'plan': '#ff9cff', 'predict': '#c6a0ff', 'trail': '#19ccf2',
    'pending': '#ffffff', 'truth_ball': '#ff2020', 'truth_self': '#7CFC00',
    'truth_opp': '#ff40ff', 'initial': '#8b949e',
}


def pose_of(obj):
    """Return (x, y, theta) of a TrackedObject."""
    return (obj.pose.x, obj.pose.y, obj.pose.theta)


def sigma_xy(obj):
    """Return the worst-axis position standard deviation of a TrackedObject [m]."""
    major, _, _ = style.covariance_ellipse(obj.covariance, sigma=1.0)
    return major


# ====================================================================== ROS side
class VerifierNode(Node):
    """Collects the latest messages for the window and sends verifier requests."""

    def __init__(self):
        super().__init__('behavior_verifier')
        p = self.declare_parameter
        bt_share = get_package_share_directory('pinky_soccer_bt')
        self.catalog_path = p('catalog', os.path.join(bt_share, 'config',
                                                      'verify_catalog.yaml')).value
        self.field_spec = p('field_spec', '').value
        self.our_goal_color = p('our_goal_color', 'yellow').value
        self.simulation = p('simulation', False).value
        self.mode_label = p('mode_label', 'real').value
        self.controller_config = p('controller_config', '').value
        self.max_lin = p('max_lin', 0.0).value
        self.max_ang = p('max_ang', 0.0).value
        self.controller_kind = p('controller_kind', 'mpc').value
        self.default_timeout = p('default_timeout', 30.0).value
        self.log_dir = os.path.expanduser(p('log_dir', '~/.ros/pinky_soccer/verify').value)
        command_topic = p('command_topic', '/cmd_vel').value
        self.lock = threading.Lock()
        self.wm = self.goal = self.status = self.models = None
        self.wm_time = self.status_time = self.goal_time = 0.0
        self.cmd = (0.0, 0.0)
        self.mpc_path, self.mpc_path_time = [], 0.0
        self.truth = {}
        self.truth_time = {}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.request_pub = self.create_publisher(String, '/behavior_test/request', reliable)
        self.playing_pub = self.create_publisher(Bool, '/game/playing', 10)
        self.create_subscription(WorldModel, '/world_model', self.on_wm, 10)
        self.create_subscription(BehaviorGoal, '/behavior_goal', self.on_goal, 10)
        self.create_subscription(Twist, command_topic, self.on_cmd, 10)
        self.create_subscription(Path, '/mpc/predicted_path', self.on_mpc_path, 10)
        self.create_subscription(String, '/behavior_test/status', self.on_status, reliable)
        self.create_subscription(String, '/behavior_test/models', self.on_models, latched)
        if self.simulation:
            self.ball_pub = self.create_publisher(Point, '/sim/set_ball', 10)
            for name, topic in (('self', '/ground_truth'), ('ball', '/sim/ball_ground_truth'),
                                ('opponent', '/sim/opponent_ground_truth')):
                self.create_subscription(PoseStamped, topic,
                                         lambda m, n=name: self.on_truth(n, m), 10)
        self.create_timer(0.25, lambda: self.send({'command': 'ping'}))

    # -------------------------------------------------------------- callbacks
    def on_wm(self, msg):
        """Keep the latest world model."""
        with self.lock:
            self.wm, self.wm_time = msg, time.monotonic()

    def on_goal(self, msg):
        """Keep the latest behaviour goal."""
        with self.lock:
            self.goal, self.goal_time = msg, time.monotonic()

    def on_cmd(self, msg):
        """Keep the latest velocity command."""
        with self.lock:
            self.cmd = (msg.linear.x, msg.angular.z)

    def on_mpc_path(self, msg):
        """Keep the NMPC predicted trajectory (map frame)."""
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        with self.lock:
            self.mpc_path, self.mpc_path_time = points, time.monotonic()

    def on_status(self, msg):
        """Keep the latest bt_node verify status."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.status, self.status_time = data, time.monotonic()

    def on_models(self, msg):
        """Keep the node models / runnable list from bt_node."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.models = data

    def on_truth(self, name, msg):
        """Store a ground-truth pose (simulation display/evaluation only)."""
        pose = (msg.pose.position.x, msg.pose.position.y,
                core.yaw_from_quaternion(msg.pose.orientation))
        with self.lock:
            self.truth[name] = pose
            self.truth_time[name] = time.monotonic()

    # -------------------------------------------------------------- output
    def send(self, request):
        """Publish one JSON request to bt_node (no-op once ROS is shut down)."""
        self._publish(self.request_pub, String(data=json.dumps(request)))

    def place_ball(self, x, y):
        """Ask the simulator to move the ball to (x, y), at rest (simulation only)."""
        if self.simulation:
            self._publish(self.ball_pub, Point(x=float(x), y=float(y), z=0.0))

    def set_playing(self, value):
        """Publish /game/playing."""
        self._publish(self.playing_pub, Bool(data=bool(value)))

    def _publish(self, pub, msg):
        # After Ctrl-C the context is gone; bt_node then stops the run by its heartbeat timeout.
        if not self.context.ok():
            return
        try:
            pub.publish(msg)
        except RCLError:
            pass

    def snapshot(self):
        """Return a consistent copy of the latest data."""
        now = time.monotonic()
        with self.lock:
            truth = {k: v for k, v in self.truth.items() if now - self.truth_time[k] < 1.0}
            return {'wm': self.wm, 'wm_age': now - self.wm_time if self.wm else None,
                    'goal': self.goal, 'goal_age': now - self.goal_time if self.goal else None,
                    'status': self.status,
                    'status_age': now - self.status_time if self.status else None,
                    'cmd': self.cmd, 'truth': truth, 'models': self.models,
                    'mpc_path': self.mpc_path if now - self.mpc_path_time < 0.5 else [],
                    't': self.get_clock().now().nanoseconds * 1e-9}


# ====================================================================== map
class MapView(tk.Canvas):
    """Field map with world <-> pixel transforms and pose picking."""

    def __init__(self, parent, field, on_pick, on_pick_run, on_right=None, **kw):
        super().__init__(parent, background=COLORS['bg'], highlightthickness=0, **kw)
        self.field = field
        walls = field.inner_walls
        t = field.spec['wall_thickness']
        margin = 0.06
        self.xmin, self.xmax = walls.xmin - t - margin, walls.xmax + t + margin
        self.ymin, self.ymax = walls.ymin - t - margin, walls.ymax + t + margin
        self.scale, self.ox, self.oy = 100.0, 0.0, 0.0
        self.on_pick, self.on_pick_run = on_pick, on_pick_run
        self.press = None
        self.drag_pose = None
        self.hover = None
        self.show_grid = tk.BooleanVar(value=False)
        self.bind('<Configure>', lambda e: self.draw_static())
        self.bind('<ButtonPress-1>', self._press)
        self.bind('<B1-Motion>', self._drag)
        self.bind('<ButtonRelease-1>', self._release)
        self.bind('<Double-Button-1>', self._double)
        if on_right is not None:
            self.bind('<ButtonPress-3>', lambda e: on_right(*self.world(e.x, e.y)))
        self.bind('<Motion>', self._motion)
        self.bind('<Leave>', lambda e: setattr(self, 'hover', None))

    # -------------------------------------------------------------- transforms
    def px(self, x, y):
        """Return the canvas pixel of a map point."""
        return (self.ox + (x - self.xmin) * self.scale, self.oy + (self.ymax - y) * self.scale)

    def world(self, u, v):
        """Return the map point of a canvas pixel."""
        return (self.xmin + (u - self.ox) / self.scale, self.ymax - (v - self.oy) / self.scale)

    def m(self, length):
        """Return a length in pixels."""
        return length * self.scale

    def rect(self, r, **kw):
        """Draw a field Rect."""
        x0, y0 = self.px(r.xmin, r.ymax)
        x1, y1 = self.px(r.xmax, r.ymin)
        return self.create_rectangle(x0, y0, x1, y1, **kw)

    def draw_static(self):
        """Recompute the transform and draw the field."""
        w, h = max(self.winfo_width(), 50), max(self.winfo_height(), 50)
        self.scale = min(w / (self.xmax - self.xmin), h / (self.ymax - self.ymin))
        self.ox = (w - (self.xmax - self.xmin) * self.scale) / 2
        self.oy = (h - (self.ymax - self.ymin) * self.scale) / 2
        self.delete('static')
        f = self.field
        t = f.spec['wall_thickness']
        walls = f.inner_walls
        x0, y0 = self.px(walls.xmin - t, walls.ymax + t)
        x1, y1 = self.px(walls.xmax + t, walls.ymin - t)
        self.create_rectangle(x0, y0, x1, y1, fill=COLORS['wall'], outline='', tags='static')
        self.rect(walls, fill=COLORS['carpet'], outline='', tags='static')
        for color in ('yellow', 'blue'):
            self.rect(f.goal_interior(color), fill=COLORS[color], outline='', tags='static')
        lw = max(1, round(self.m(f.spec['line_width'])))
        self.rect(f.playing_area, outline=COLORS['line'], width=lw, tags='static')
        for color in ('yellow', 'blue'):
            self.rect(f.penalty_area(color), outline=COLORS['line'], width=lw, tags='static')
            (gx0, gy0), (gx1, gy1) = f.goal_mouth(color)
            self.create_line(*self.px(gx0, gy0), *self.px(gx1, gy1), fill=COLORS[color],
                             width=lw + 2, tags='static')
        cx, cy, r = f.center_circle
        u, v = self.px(cx, cy)
        self.create_oval(u - self.m(r), v - self.m(r), u + self.m(r), v + self.m(r),
                         outline=COLORS['line'], tags='static')
        for x, y in f.neutral_spots:
            u, v = self.px(x, y)
            self.create_oval(u - 2, v - 2, u + 2, v + 2, fill=COLORS['line'], outline='',
                             tags='static')
        if self.show_grid.get():
            step = 0.25
            k0, k1 = math.floor(walls.xmin / step), math.ceil(walls.xmax / step)
            for k in range(k0, k1 + 1):
                self.create_line(*self.px(k * step, walls.ymin), *self.px(k * step, walls.ymax),
                                 fill='#3d8a4c', dash=(1, 3), tags='static')
            k0, k1 = math.floor(walls.ymin / step), math.ceil(walls.ymax / step)
            for k in range(k0, k1 + 1):
                self.create_line(*self.px(walls.xmin, k * step), *self.px(walls.xmax, k * step),
                                 fill='#3d8a4c', dash=(1, 3), tags='static')
        # Axis hint: +X towards the blue goal.
        ax, ay = walls.xmin + 0.05, walls.ymin + 0.05
        self.create_line(*self.px(ax, ay), *self.px(ax + 0.15, ay), arrow=tk.LAST,
                         fill=COLORS['line'], tags='static')
        self.create_line(*self.px(ax, ay), *self.px(ax, ay + 0.15), arrow=tk.LAST,
                         fill=COLORS['line'], tags='static')
        self.create_text(*self.px(ax + 0.17, ay), text='+X', fill=COLORS['line'], anchor='w',
                         tags='static', font=('TkDefaultFont', 8))
        self.create_text(*self.px(ax, ay + 0.17), text='+Y', fill=COLORS['line'], anchor='s',
                         tags='static', font=('TkDefaultFont', 8))
        self.tag_lower('static')

    # -------------------------------------------------------------- primitives
    def arrow(self, pose, length, color, width=2, dash=None, tags='dyn'):
        """Arrow starting at the pose position, pointing along its heading."""
        x, y, th = pose
        return self.create_line(*self.px(x, y),
                                *self.px(x + length * math.cos(th), y + length * math.sin(th)),
                                arrow=tk.LAST, fill=color, width=width, dash=dash,
                                arrowshape=(10, 12, 4), tags=tags)

    def circle(self, x, y, radius, min_px=3, **kw):
        """Circle with a minimum pixel radius."""
        u, v = self.px(x, y)
        r = max(min_px, self.m(radius))
        return self.create_oval(u - r, v - r, u + r, v + r, tags='dyn', **kw)

    def polyline(self, points, **kw):
        """Line through map points (needs two or more)."""
        if len(points) < 2:
            return None
        flat = [c for p in points for c in self.px(*p[:2])]
        return self.create_line(*flat, tags='dyn', **kw)

    def ellipse(self, center, cov, color):
        """2-sigma covariance ellipse."""
        major, minor, angle = style.covariance_ellipse(cov)
        if major <= 1e-4:
            return
        pts = []
        for k in range(36):
            a = 2 * math.pi * k / 36
            ex, ey = major * math.cos(a), minor * math.sin(a)
            pts.append((center[0] + ex * math.cos(angle) - ey * math.sin(angle),
                        center[1] + ex * math.sin(angle) + ey * math.cos(angle)))
        self.polyline(pts + pts[:1], fill=color, dash=(2, 2))

    def cross(self, x, y, size, color):
        """X mark (lost objects)."""
        self.polyline([(x - size, y - size), (x + size, y + size)], fill=color, width=2)
        self.polyline([(x - size, y + size), (x + size, y - size)], fill=color, width=2)

    # -------------------------------------------------------------- picking
    def _snap(self, event):
        return bool(event.state & 0x1)

    def _press(self, event):
        self.press = self.world(event.x, event.y)
        self.drag_pose = None

    def _drag(self, event):
        if self.press is not None:
            self.drag_pose = core.drag_pose(self.press, self.world(event.x, event.y), None,
                                            snap=self._snap(event))
        self._motion(event)

    def _release(self, event):
        if self.press is None:
            return
        pose = core.drag_pose(self.press, self.world(event.x, event.y), None,
                              snap=self._snap(event))
        self.press, self.drag_pose = None, None
        self.on_pick(pose)

    def _double(self, event):
        self.on_pick_run()

    def _motion(self, event):
        self.hover = self.world(event.x, event.y)


# ====================================================================== window
class VerifierApp:
    """Tk window: catalog list, details/controls and the map."""

    def __init__(self, root, node):
        self.root, self.node = root, node
        self.catalog = core.load_catalog(node.catalog_path)
        self.items = {item['id']: item for item in self.catalog['items']}
        self.field = Field.from_file(node.field_spec or None, node.our_goal_color)
        overrides = {k: v for k, v in (('max_lin', node.max_lin), ('max_ang', node.max_ang))
                     if v and v > 0}
        self.gains = core.load_gains(node.controller_config, overrides)
        self.session_dir = os.path.join(
            node.log_dir, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.selected = None
        self.port_vars = {}
        self.input_labels = {}
        self.record = None
        self.records = []
        self.run_number = 0
        self.last_run = None
        self.playing_sent = False
        self.probe_sent = 0.0
        self.sent_time = 0.0
        self.trail, self.truth_trail = [], []
        self.prediction = []
        self.done_streak = 0
        self.snap = {}
        self.quit_requested = False
        self._build()
        self.select(self.catalog['items'][0]['id'])
        self.root.after(100, self.refresh)

    # -------------------------------------------------------------- layout
    def _build(self):
        r = self.root
        r.title(f'Pinky 행동 검증 [{self.node.mode_label}]')
        w = min(1800, int(0.95 * r.winfo_screenwidth()))
        h = min(1050, int(0.9 * r.winfo_screenheight()))
        r.geometry(f'{w}x{h}')
        r.configure(background=COLORS['bg'])
        st = ttk.Style(r)
        st.theme_use('clam')
        for name in ('TFrame', 'TLabelframe', 'TPanedwindow'):
            st.configure(name, background=COLORS['panel'])
        st.configure('TLabelframe.Label', background=COLORS['panel'], foreground=COLORS['text'])
        st.configure('TLabel', background=COLORS['panel'], foreground=COLORS['text'])
        st.configure('Muted.TLabel', foreground=COLORS['muted'])
        st.configure('Title.TLabel', font=('TkDefaultFont', 15, 'bold'))
        st.configure('State.TLabel', font=('TkDefaultFont', 12, 'bold'))
        st.configure('TCheckbutton', background=COLORS['panel'], foreground=COLORS['text'])
        st.configure('Treeview', background='#1b1f24', fieldbackground='#1b1f24',
                     foreground=COLORS['text'], rowheight=22)
        st.configure('TNotebook', background=COLORS['panel'])
        st.configure('Run.TButton', font=('TkDefaultFont', 11, 'bold'), foreground='#0d4d1d')
        st.configure('Stop.TButton', font=('TkDefaultFont', 11, 'bold'), foreground='#8a0f0f')

        top = ttk.Frame(r, padding=(8, 4))
        top.pack(fill='x')
        mode = self.node.mode_label.upper()
        ctl = 'NMPC' if self.node.controller_kind == 'mpc' else '비례 제어'
        ttk.Label(top, text=f'모드: {mode} · 제어: {ctl}', style='State.TLabel').pack(side='left')
        self.conn_label = ttk.Label(top, text='', style='Muted.TLabel')
        self.conn_label.pack(side='left', padx=16)
        ttk.Button(top, text='■ 비상 정지 (Esc)', style='Stop.TButton', takefocus=0,
                   command=self.stop).pack(side='right')
        if mode == 'REAL':
            ttk.Label(top, text='실물: 사람이 옆에서 전원/정지 준비', foreground='#ffb000'
                      ).pack(side='right', padx=12)

        panes = ttk.PanedWindow(r, orient='horizontal')
        panes.pack(fill='both', expand=True)
        panes.add(self._build_list(panes), weight=0)
        panes.add(self._build_details(panes), weight=0)
        panes.add(self._build_map(panes), weight=1)

        r.bind_all('<Return>', lambda e: self.run())
        r.bind_all('<KP_Enter>', lambda e: self.run())
        r.bind_all('<Escape>', lambda e: self.stop())
        r.bind_all('<space>', self._space)
        r.bind_all('<F5>', lambda e: self.repeat())
        r.protocol('WM_DELETE_WINDOW', self.close)

    def _build_list(self, parent):
        frame = ttk.Frame(parent, padding=6)
        ttk.Label(frame, text='행동 / 스킬', style='State.TLabel').pack(anchor='w')
        self.tree = ttk.Treeview(frame, show='tree', selectmode='browse', height=30)
        self.tree.column('#0', width=230)
        self.tree.pack(fill='both', expand=True, pady=4)
        for group, items in core.groups(self.catalog):
            gid = self.tree.insert('', 'end', text=group, open=True)
            for item in items:
                self.tree.insert(gid, 'end', iid=item['id'], text=f'{item["title"]}')
        self.tree.bind('<<TreeviewSelect>>', self._on_tree_select)
        self.tree.bind('<Double-1>', lambda e: self.run())
        ttk.Label(frame, style='Muted.TLabel', justify='left', text=(
            'Enter 실행 · Esc/Space 정지 · F5 다시\n'
            '지도: 클릭=위치, 끌기=위치+방향\n'
            '  Shift=15° 단위, 더블클릭=설정 후 실행' +
            ('\n  우클릭=공 옮기기(시뮬 전용)' if self.node.simulation else ''))).pack(anchor='w')
        return frame

    def _build_details(self, parent):
        outer = ttk.Frame(parent, padding=6, width=430)
        self.title_label = ttk.Label(outer, text='', style='Title.TLabel')
        self.title_label.pack(anchor='w')
        self.id_label = ttk.Label(outer, text='', style='Muted.TLabel')
        self.id_label.pack(anchor='w')
        self.text_label = ttk.Label(outer, text='', wraplength=410, justify='left')
        self.text_label.pack(anchor='w', pady=(4, 6))

        self.ports_frame = ttk.LabelFrame(outer, text='입력 (수정 가능)', padding=6)
        self.ports_frame.pack(fill='x')
        self.inputs_frame = ttk.LabelFrame(outer, text='자동 입력 (읽기 전용, 현재 값)',
                                           padding=6)
        self.inputs_frame.pack(fill='x', pady=4)
        self.result_label = ttk.Label(outer, text='', wraplength=410, justify='left',
                                      style='Muted.TLabel')
        self.result_label.pack(anchor='w', pady=(0, 4))

        ctl = ttk.Frame(outer)
        ctl.pack(fill='x', pady=4)
        ttk.Button(ctl, text='▶ 실행', style='Run.TButton', takefocus=0,
                   command=self.run).pack(side='left')
        ttk.Button(ctl, text='■ 정지', style='Stop.TButton', takefocus=0,
                   command=self.stop).pack(side='left', padx=4)
        ttk.Button(ctl, text='↻ 다시', takefocus=0, command=self.repeat).pack(side='left')
        ttk.Button(ctl, text='기본값', takefocus=0, command=self.reset_ports).pack(side='left',
                                                                                padx=4)
        ttk.Label(ctl, text='제한(s)').pack(side='left', padx=(8, 2))
        self.timeout_var = tk.StringVar(value=core.format_port(self.node.default_timeout))
        ttk.Spinbox(ctl, from_=1, to=600, increment=5, width=5,
                    textvariable=self.timeout_var).pack(side='left')
        self.auto_done = tk.BooleanVar(value=True)
        ttk.Checkbutton(outer, text='완료 조건(예: IsBehindBall, 득점)에서 자동 종료',
                        variable=self.auto_done, takefocus=0).pack(anchor='w')

        self.state_label = ttk.Label(outer, text='대기', style='State.TLabel')
        self.state_label.pack(anchor='w', pady=(6, 0))
        self.goal_label = ttk.Label(outer, text='', style='Muted.TLabel', wraplength=410,
                                    justify='left')
        self.goal_label.pack(anchor='w')

        return outer

    def _build_tabs(self, parent):
        nb = ttk.Notebook(parent)
        self.nodes_view = self._treeview(nb, ('상태', '최근'), (90, 80), '노드', 260)
        nb.add(self.nodes_view.master, text='노드 상태')
        self.cond_view = self._treeview(nb, ('값',), (70,), '조건', 420)
        nb.add(self.cond_view.master, text='조건')
        self.metric_view = self._treeview(nb, ('값',), (360,), '항목', 200)
        nb.add(self.metric_view.master, text='평가')
        self.hist_view = self._treeview(nb, ('결과', '시간', '핵심'), (200, 70, 110), '#  행동',
                                        180)
        self.hist_view.bind('<Double-1>', self._history_load)
        nb.add(self.hist_view.master, text='기록')
        self.notebook = nb
        for status, color in STATUS_COLORS.items():
            self.nodes_view.tag_configure(status, foreground=color)
        self.cond_view.tag_configure('true', foreground=STATUS_COLORS['SUCCESS'])
        self.cond_view.tag_configure('false', foreground=STATUS_COLORS['FAILURE'])
        self.cond_view.tag_configure('none', foreground=STATUS_COLORS['IDLE'])
        self.log_label = ttk.Label(parent, text=f'기록 폴더: {self.session_dir}',
                                   style='Muted.TLabel')
        return nb

    def _treeview(self, parent, columns, widths, first, first_width):
        frame = ttk.Frame(parent)
        view = ttk.Treeview(frame, columns=columns, height=9)
        view.heading('#0', text=first)
        view.column('#0', width=first_width)
        for c, w in zip(columns, widths):
            view.heading(c, text=c)
            view.column(c, width=w)
        view.pack(fill='both', expand=True)
        return view

    def _build_map(self, parent):
        column = ttk.PanedWindow(parent, orient='vertical')
        frame = ttk.Frame(column, padding=4)
        column.add(frame, weight=3)
        bottom = ttk.Frame(column, padding=(4, 0, 4, 4))
        column.add(bottom, weight=1)
        self._build_tabs(bottom).pack(fill='both', expand=True)
        self.log_label.pack(anchor='w')
        bar = ttk.Frame(frame)
        bar.pack(fill='x')
        self.show_truth = tk.BooleanVar(value=self.node.simulation)
        self.show_trail = tk.BooleanVar(value=True)
        self.show_predict = tk.BooleanVar(value=True)
        self.show_cov = tk.BooleanVar(value=True)
        truth_box = ttk.Checkbutton(bar, text='정답 위치 (시뮬)', variable=self.show_truth,
                                    takefocus=0)
        truth_box.pack(side='left')
        if not self.node.simulation:
            truth_box.state(['disabled'])
            self.show_truth.set(False)
        for text, var in (('지나온 길', self.show_trail), ('예상 경로', self.show_predict),
                          ('불확실성', self.show_cov)):
            box = ttk.Checkbutton(bar, text=text, variable=var, takefocus=0)
            box.pack(side='left', padx=4)
        self.map = MapView(frame, self.field, self.on_pick, self.on_pick_run,
                           self.on_place_ball if self.node.simulation else None)
        ttk.Checkbutton(bar, text='격자 0.25m', variable=self.map.show_grid, takefocus=0,
                        command=self.map.draw_static).pack(side='left', padx=4)
        ttk.Button(bar, text='길 지우기', takefocus=0, command=self.clear_trail).pack(side='right')
        self.map.pack(fill='both', expand=True, pady=4)
        self.hover_label = ttk.Label(frame, text='', style='Muted.TLabel')
        self.hover_label.pack(anchor='w')
        legend = ('추정: ● 내 로봇(하늘) · 공(주황) · ■ 상대(빨강)  실선=관측 점선=추정 X=분실 · '
                  '목표(자홍) · 계획 경유점(분홍) · 예상 경로(보라 점선, NMPC 예측) · 시작 시 예상(회색)')
        if self.node.simulation:
            legend += '\n정답: 공=빨간 점 · 나=연두 화살표 · 상대=분홍 화살표'
        legend_label = ttk.Label(frame, text=legend, style='Muted.TLabel', justify='left')
        legend_label.pack(anchor='w', fill='x')
        frame.bind('<Configure>', lambda e: legend_label.configure(
            wraplength=max(200, e.width - 12)))
        return column

    # -------------------------------------------------------------- selection
    def _on_tree_select(self, _event):
        sel = self.tree.selection()
        if sel and sel[0] in self.items:
            self.select(sel[0])

    def select(self, item_id, values=None):
        """Show an item; optionally load port values."""
        item = self.items[item_id]
        self.selected = item
        if self.tree.selection() != (item_id,):
            self.tree.selection_set(item_id)
            self.tree.see(item_id)
        self.title_label.configure(text=item['title'])
        kind = '단일 행동 노드' if item['kind'] == 'action' else '서브트리 (soccer_1v1.xml)'
        self.id_label.configure(text=f'{item["id"]} · {kind}')
        self.text_label.configure(text=' '.join(item['text'].split()))
        self.result_label.configure(text='결과: ' + ' '.join(item['result'].split()))
        for w in self.ports_frame.winfo_children():
            w.destroy()
        self.port_vars = {}
        if not item['ports']:
            ttk.Label(self.ports_frame, text='설정할 값 없음', style='Muted.TLabel').grid(
                row=0, column=0, sticky='w')
        for row, port in enumerate(item['ports']):
            var = tk.StringVar(value=core.format_port(
                (values or {}).get(port['name'], port['default'])))
            self.port_vars[port['name']] = var
            ttk.Label(self.ports_frame, text=port['label']).grid(row=2 * row, column=0,
                                                                 sticky='w')
            ttk.Spinbox(self.ports_frame, from_=port['min'], to=port['max'],
                        increment=port['step'], width=8, textvariable=var).grid(
                row=2 * row, column=1, padx=4)
            ttk.Label(self.ports_frame, text=f'{port["unit"]}  (기본 {port["default"]})',
                      style='Muted.TLabel').grid(row=2 * row, column=2, sticky='w')
            ttk.Label(self.ports_frame, text=f'{port["name"]}: {port["text"]}',
                      style='Muted.TLabel', wraplength=390).grid(
                row=2 * row + 1, column=0, columnspan=3, sticky='w', pady=(0, 3))
        if item.get('pose_input'):
            ttk.Label(self.ports_frame, text='→ 지도에서 클릭/끌어서 입력', foreground='#ffd27f'
                      ).grid(row=2 * len(item['ports']), column=0, columnspan=3, sticky='w')
        for w in self.inputs_frame.winfo_children():
            w.destroy()
        self.input_labels = {}
        world = self.catalog.get('world_inputs', {})
        if not item['inputs']:
            ttk.Label(self.inputs_frame, text='없음', style='Muted.TLabel').grid(row=0, column=0)
        for row, key in enumerate(item['inputs']):
            info = world.get(key, {'label': key, 'text': ''})
            ttk.Label(self.inputs_frame, text=info['label']).grid(row=2 * row, column=0,
                                                                  sticky='nw')
            value = ttk.Label(self.inputs_frame, text='-', foreground='#9fe1ff')
            value.grid(row=2 * row, column=1, sticky='w', padx=6)
            self.input_labels[key] = value
            ttk.Label(self.inputs_frame, text=info['text'], style='Muted.TLabel',
                      wraplength=390).grid(row=2 * row + 1, column=0, columnspan=2, sticky='w')

    def port_values(self):
        """Return the current port values as floats (raises ValueError)."""
        out = {}
        for name, var in self.port_vars.items():
            try:
                out[name] = float(var.get())
            except ValueError:
                raise ValueError(f'{name}: 숫자를 입력하세요') from None
        return out

    def reset_ports(self):
        """Restore catalog defaults."""
        for port in self.selected['ports']:
            self.port_vars[port['name']].set(core.format_port(port['default']))

    def on_pick(self, pose):
        """Map click/drag: fill the pose ports of the selected item."""
        mapping = self.selected.get('pose_input')
        if not mapping:
            return
        x, y, th = pose
        if th is None:
            try:
                th = float(self.port_vars[mapping['theta']].get())
            except ValueError:
                th = 0.0
        ports = {p['name']: p for p in self.selected['ports']}
        for key, value in (('x', x), ('y', y), ('theta', th)):
            port = ports[mapping[key]]
            value = min(port['max'], max(port['min'], value))
            self.port_vars[mapping[key]].set(core.format_port(round(value, 3)))

    def on_place_ball(self, x, y):
        """Right-click in simulation: move the ball there."""
        walls = self.field.inner_walls
        if walls.contains(x, y, margin=0.03):
            self.node.place_ball(x, y)
            self.state_label.configure(text=f'공 옮김 ({x:+.2f}, {y:+.2f}) — 시뮬 전용',
                                       foreground=COLORS['text'])

    def on_pick_run(self):
        """Double-click: the first click already set the pose, run it."""
        if self.selected.get('pose_input'):
            self.run()

    def _space(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Spinbox)):
            return
        self.stop()

    # -------------------------------------------------------------- run control
    def run(self, item=None, values=None):
        """Send a run request for the selected (or given) item."""
        item = item or self.selected
        try:
            values = values if values is not None else self.port_values()
            timeout = float(self.timeout_var.get())
            request = core.run_request(self.run_number + 1, item, values, timeout)
        except ValueError as exc:
            self.state_label.configure(text=f'입력 오류: {exc}', foreground='#f85149')
            return
        if self.record is not None and not self.record.done:
            self._end_run('stopped', '새 실행으로 교체')
        self.run_number += 1
        if item.get('playing'):
            self.node.set_playing(True)
            self.playing_sent = True
        self.node.send(request)
        self.sent_time = time.monotonic()
        t = self.snap.get('t', 0.0)
        self.record = core.RunRecord(self.run_number, item, request['ports'], t, self.field)
        self.last_run = (item['id'], values)
        self.done_streak = 0
        self.trail, self.truth_trail = [], []
        self.state_label.configure(text=f'#{self.run_number} {item["title"]} 요청',
                                   foreground=COLORS['text'])

    def repeat(self):
        """Run the last request again."""
        if self.last_run:
            item_id, values = self.last_run
            self.select(item_id, values)
            self.run(self.items[item_id], values)

    def stop(self):
        """Stop whatever runs (always sends STOP)."""
        self.node.send({'command': 'stop'})
        if self.playing_sent:
            self.node.set_playing(False)
            self.playing_sent = False
        if self.record is not None and not self.record.done:
            self._end_run('stopped', '')

    def _end_run(self, state, message):
        rec = self.record
        rec.finish(self.snap.get('t', rec.start), state, message)
        if self.playing_sent:
            self.node.set_playing(False)
            self.playing_sent = False
        try:
            rec.write(self.session_dir)
        except OSError as exc:
            self.log_label.configure(text=f'기록 저장 실패: {exc}')
        self.records.append(rec)
        result, duration, key = rec.summary()
        ok = state in ('success', 'done_condition')
        self.state_label.configure(
            text=f'#{rec.number} {rec.item["title"]}: {result} · {duration}',
            foreground=STATUS_COLORS['SUCCESS'] if ok else (
                COLORS['text'] if state == 'stopped' else STATUS_COLORS['FAILURE']))
        self.notebook.select(2)
        self.hist_view.insert('', 0, iid=str(rec.number),
                              text=f'{rec.number}  {rec.item["title"]}',
                              values=(result, duration, key))
        self.show_metrics(rec)

    def _history_load(self, _event):
        sel = self.hist_view.selection()
        if not sel:
            return
        rec = next((r for r in self.records if str(r.number) == sel[0]), None)
        if rec:
            self.select(rec.item['id'], {k: float(v) for k, v in rec.ports.items()})
            self.show_metrics(rec)

    def show_metrics(self, rec):
        """Fill the evaluation tab."""
        self.metric_view.delete(*self.metric_view.get_children())
        self.metric_view.insert('', 'end', text='실행', values=(
            f'#{rec.number} {rec.item["id"]} ' +
            ' '.join(f'{k}={v}' for k, v in rec.ports.items()),))
        for label, value in rec.metrics():
            self.metric_view.insert('', 'end', text=label, values=(value,))

    def clear_trail(self):
        """Forget the drawn trails."""
        self.trail, self.truth_trail = [], []

    def close(self):
        """Stop the robot, then close."""
        try:
            self.stop()
            time.sleep(0.1)
        finally:
            self.root.destroy()

    # -------------------------------------------------------------- periodic update
    def refresh(self):
        """Pull ROS data, update run state and redraw (about 15 Hz)."""
        if self.quit_requested or not self.node.context.ok():
            self.close()
            return
        try:
            self.snap = s = self.node.snapshot()
            self._update_connection(s)
            self._update_run(s)
            self._update_inputs(s)
            self._update_status_views(s)
            self._draw(s)
        finally:
            self.root.after(66, self.refresh)

    def _update_connection(self, s):
        parts = []
        wm_age, st_age = s['wm_age'], s['status_age']
        parts.append('월드모델 ' + ('없음' if wm_age is None else
                                ('정상' if wm_age < 0.5 else f'끊김 {wm_age:.1f}s')))
        bt_ok = st_age is not None and st_age < 1.0
        parts.append('bt_node 검증모드 ' + ('정상' if bt_ok else '응답 없음'))
        if self.node.simulation:
            truth = s['truth']
            parts.append('정답 ' + (', '.join(sorted(truth)) if truth else '없음'))
        models = s['models']
        if models:
            missing = set(self.items) - set(models.get('runnable', []))
            if missing:
                parts.append('bt_node에 없는 항목: ' + ', '.join(sorted(missing)))
        self.conn_label.configure(text=' · '.join(parts),
                                  foreground='#9aa4ae' if bt_ok else '#f85149')
        status = s['status']
        now = time.monotonic()
        if bt_ok and self.catalog.get('probe') and not (status or {}).get('conditions') \
                and now - self.probe_sent > 2.0:
            self.node.send(core.probe_request(self.catalog))
            self.probe_sent = now

    def _update_run(self, s):
        rec = self.record
        if rec is None or rec.done:
            return
        status = s['status'] or {}
        wm, goal = s['wm'], s['goal']
        est = pose_of(wm.self) if wm else None
        ball = pose_of(wm.ball)[:2] if wm else None
        truth = s['truth']
        goal_tuple = None
        if goal is not None:
            goal_tuple = (goal.action, (goal.target.x, goal.target.y, goal.target.theta))
        rec.add(s['t'], est, truth.get('self'), ball, wm.ball.state if wm else None,
                truth.get('ball', (None, None))[:2] if 'ball' in truth else None,
                s['cmd'], goal_tuple)
        if status.get('request_id') == rec.number:
            state = status.get('state')
            text = core.STATE_TEXT.get(state, state)
            self.state_label.configure(text=f'#{rec.number} {rec.item["title"]}: {text} '
                                       f'({status.get("elapsed", 0):.1f}s)',
                                       foreground=STATUS_COLORS['RUNNING'] if state == 'running'
                                       else COLORS['text'])
            if state in core.TERMINAL:
                self._end_run(state, status.get('message', ''))
                return
            if not rec.initial_prediction and self.prediction:
                rec.initial_prediction = list(self.prediction)
        elif time.monotonic() - self.sent_time > 2.0:
            self._end_run('error', 'bt_node 응답 없음 (verify_mode:=true 인지 확인)')
            return
        if self.auto_done.get():
            done = rec.item.get('done_when')
            hit = False
            if done == 'goal_scored':
                hit = rec.goal_scored
            elif done:
                hit = any(c.get('name') == done and c.get('value') is True
                          for c in status.get('conditions', []))
            self.done_streak = self.done_streak + 1 if hit else 0
            if self.done_streak >= 2:
                self.node.send({'command': 'stop'})
                self._end_run('done_condition', done)

    def _update_inputs(self, s):
        wm = s['wm']
        if wm is None:
            for label in self.input_labels.values():
                label.configure(text='(월드 모델 없음)')
            return
        texts = {
            'pose': (f'x {wm.self.pose.x:+.3f}  y {wm.self.pose.y:+.3f}  '
                     f'θ {math.degrees(wm.self.pose.theta):+.1f}°  '
                     f'{STATE_NAMES.get(wm.self.state, "?")} σ{100 * sigma_xy(wm.self):.1f}cm'),
            'ball': (f'x {wm.ball.pose.x:+.3f}  y {wm.ball.pose.y:+.3f}  '
                     f'{STATE_NAMES.get(wm.ball.state, "?")} σ{100 * sigma_xy(wm.ball):.1f}cm'),
            'opponents': (f'{len(wm.opponents)}대 ' + ' '.join(
                f'({o.pose.x:+.2f},{o.pose.y:+.2f} {STATE_NAMES.get(o.state, "?")[0]})'
                for o in wm.opponents[:3])),
            'own_goal': f'x {wm.own_goal.x:+.3f}  y {wm.own_goal.y:+.3f}',
            'opp_goal': f'x {wm.opp_goal.x:+.3f}  y {wm.opp_goal.y:+.3f}',
            'playing': '예' if wm.playing else '아니오',
        }
        for key, label in self.input_labels.items():
            label.configure(text=texts.get(key, '-'))

    def _update_status_views(self, s):
        status = s['status'] or {}
        goal = s['goal']
        if goal is not None:
            mode = {0: 'NAVIGATE', 1: 'REACTIVE', 2: 'STOP'}.get(goal.mode, goal.mode)
            v, w = s['cmd']
            self.goal_label.configure(text=(
                f'목표: {goal.branch}/{goal.action} {mode}  '
                f'({goal.target.x:+.2f}, {goal.target.y:+.2f}, '
                f'{math.degrees(goal.target.theta):+.0f}°)  경유점 {len(goal.planned_path.poses)}개'
                f'\n명령: v {v:+.3f} m/s  ω {w:+.2f} rad/s'))
        nodes = status.get('nodes', [])
        view = self.nodes_view
        view.delete(*view.get_children())
        for n in nodes:
            st = n.get('status', 'IDLE')
            last = n.get('last', '')
            tag = st if st != 'IDLE' else (last or 'IDLE')
            view.insert('', 'end', text='  ' * n['depth'] + n['name'], values=(st, last),
                        tags=(tag,))
        cview = self.cond_view
        cview.delete(*cview.get_children())
        texts = {p['label']: p.get('text', '') for p in self.catalog.get('probe', [])}
        for c in status.get('conditions', []):
            value = c.get('value')
            tag = 'none' if value is None else ('true' if value else 'false')
            cview.insert('', 'end', text=c['name'],
                         values=({'none': '-', 'true': '참', 'false': '거짓'}[tag],), tags=(tag,))
            if texts.get(c['name']):
                cview.insert('', 'end', text='   ' + texts[c['name']], tags=('none',))
        rec = self.record
        if rec is not None and not rec.done and len(rec.rows) % 8 == 0:
            self.show_metrics(rec)

    # -------------------------------------------------------------- drawing
    def _draw(self, s):
        mp = self.map
        mp.delete('dyn')
        wm, truth = s['wm'], s['truth']
        if wm is not None:
            est = pose_of(wm.self)
            if not self.trail or math.hypot(est[0] - self.trail[-1][0],
                                            est[1] - self.trail[-1][1]) > 0.01:
                self.trail.append(est[:2])
                self.trail = self.trail[-3000:]
        if 'self' in truth:
            t = truth['self']
            if not self.truth_trail or math.hypot(t[0] - self.truth_trail[-1][0],
                                                  t[1] - self.truth_trail[-1][1]) > 0.01:
                self.truth_trail.append(t[:2])
                self.truth_trail = self.truth_trail[-3000:]
        if self.show_trail.get():
            mp.polyline(self.trail, fill=COLORS['trail'], width=1)
            if self.show_truth.get():
                mp.polyline(self.truth_trail, fill=COLORS['truth_self'], width=1, dash=(1, 2))
        rec = self.record
        if rec is not None and rec.initial_prediction and self.show_predict.get():
            mp.polyline(rec.initial_prediction, fill=COLORS['initial'], dash=(6, 4))
        self._draw_goal(s)
        self._draw_pending()
        if wm is not None:
            self._draw_world(wm)
        if self.show_truth.get():
            self._draw_truth(truth)
        if mp.drag_pose is not None:
            mp.arrow(mp.drag_pose[:2] + (mp.drag_pose[2] or 0.0,), 0.15, COLORS['pending'],
                     width=3)
        if mp.hover is not None:
            hx, hy = mp.hover
            text = f'마우스 x {hx:+.3f}  y {hy:+.3f} m'
            if wm is not None:
                d = math.hypot(hx - wm.self.pose.x, hy - wm.self.pose.y)
                text += f'   로봇에서 {d:.3f} m'
            self.hover_label.configure(text=text)

    def _draw_goal(self, s):
        mp, goal, wm = self.map, s['goal'], s['wm']
        self.prediction = []
        if goal is None or goal.mode == BehaviorGoal.STOP or (s['goal_age'] or 9) > 1.0:
            return
        waypoints = [(p.pose.position.x, p.pose.position.y,
                      core.yaw_from_quaternion(p.pose.orientation))
                     for p in goal.planned_path.poses]
        if not waypoints:
            waypoints = [(goal.target.x, goal.target.y, goal.target.theta)]
        if wm is not None:
            start = pose_of(wm.self)
            if self.show_predict.get():
                # NMPC: its own optimal trajectory; reactive: simulate the P controller law.
                self.prediction = s['mpc_path'] or core.predict_path(
                    start, waypoints, self.gains, float(goal.max_speed))
                mp.polyline(self.prediction, fill=COLORS['predict'], width=2, dash=(4, 3))
            mp.polyline([start[:2]] + [w[:2] for w in waypoints], fill=COLORS['plan'],
                        dash=(2, 4))
        for wp in waypoints[:-1]:
            mp.circle(wp[0], wp[1], 0.015, outline=COLORS['plan'], width=2)
        t = (goal.target.x, goal.target.y, goal.target.theta)
        final = waypoints[-1]
        size = 0.035
        mp.polyline([(final[0] - size, final[1]), (final[0] + size, final[1])],
                    fill=COLORS['target'], width=2)
        mp.polyline([(final[0], final[1] - size), (final[0], final[1] + size)],
                    fill=COLORS['target'], width=2)
        mp.arrow(final, 0.10, COLORS['target'], width=2)
        if (t[0], t[1]) != final[:2]:
            mp.circle(t[0], t[1], 0.02, outline=COLORS['target'], width=2)

    def _draw_pending(self):
        mapping = self.selected.get('pose_input') if self.selected else None
        if not mapping:
            return
        try:
            pose = tuple(float(self.port_vars[mapping[k]].get()) for k in ('x', 'y', 'theta'))
        except (ValueError, KeyError):
            return
        self.map.circle(pose[0], pose[1], 0.03, outline=COLORS['pending'], dash=(3, 2))
        self.map.arrow(pose, 0.12, COLORS['pending'], width=2, dash=(4, 2))
        u, v = self.map.px(pose[0], pose[1])
        self.map.create_text(u + 8, v - 10, text='입력 목표', fill=COLORS['pending'],
                             anchor='w', tags='dyn', font=('TkDefaultFont', 8))

    def _draw_world(self, wm):
        mp = self.map
        cov = self.show_cov.get()
        for opp in wm.opponents:
            if opp.state == 2:
                continue
            x, y, th = pose_of(opp)
            half = style.OPPONENT_SIZE / 2
            dash = None if opp.state == 0 else (3, 2)
            mp.polyline([(x - half, y - half), (x + half, y - half), (x + half, y + half),
                         (x - half, y + half), (x - half, y - half)], fill=COLORS['opponent'],
                        width=2, dash=dash)
            mp.arrow((x, y, th), 0.1, COLORS['opponent'], dash=dash)
            if cov:
                mp.ellipse((x, y), opp.covariance, COLORS['opponent'])
        b = wm.ball
        if b.state == 2:
            mp.cross(b.pose.x, b.pose.y, 0.025, COLORS['ball'])
        else:
            fill = COLORS['ball'] if b.state == 0 else ''
            mp.circle(b.pose.x, b.pose.y, 0.02135, min_px=4, outline=COLORS['ball'], fill=fill,
                      width=2, dash=None if b.state == 0 else (2, 2))
            if cov:
                mp.ellipse((b.pose.x, b.pose.y), b.covariance, COLORS['ball'])
            vx, vy = b.velocity.x, b.velocity.y
            if math.hypot(vx, vy) > 0.02:
                mp.polyline([(b.pose.x, b.pose.y), (b.pose.x + vx, b.pose.y + vy)],
                            fill=COLORS['ball'], dash=(1, 2), arrow=tk.LAST)
        x, y, th = pose_of(wm.self)
        dash = None if wm.self.state == 0 else (3, 2)
        mp.circle(x, y, style.ROBOT_RADIUS, outline=COLORS['self'], width=2, dash=dash)
        mp.arrow((x, y, th), 0.11, COLORS['self'], width=3)
        if cov:
            mp.ellipse((x, y), wm.self.covariance, COLORS['self'])

    def _draw_truth(self, truth):
        mp = self.map
        if 'ball' in truth:
            u, v = mp.px(*truth['ball'][:2])
            mp.create_oval(u - 3, v - 3, u + 3, v + 3, fill=COLORS['truth_ball'], outline='',
                           tags='dyn')
        if 'self' in truth:
            mp.arrow(truth['self'], 0.13, COLORS['truth_self'], width=2)
        if 'opponent' in truth:
            mp.arrow(truth['opponent'], 0.13, COLORS['truth_opp'], width=2)


def main(args=None):
    """Run the verifier window (Tk in the main thread, ROS in a background thread)."""
    # Own SIGINT/SIGTERM handling: rclpy's default handler would shut ROS down while the Tk
    # loop keeps running, and STOP could no longer be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VerifierNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    root = tk.Tk()
    app = VerifierApp(root, node)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(app, 'quit_requested', True))
    try:
        root.mainloop()
    finally:
        node.send({'command': 'stop'})
        executor.shutdown()
        spinner.join(timeout=2.0)   # a live spin thread at exit aborts the interpreter
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
