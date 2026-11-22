"""Tk sensor overview, robot-frame scan plot and complete ROS message inspection."""

import math
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from rosidl_runtime_py.convert import message_to_yaml

from .sensor_monitor import MONITOR_TOPICS, readings, scan_points, summary


class SensorMonitor(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, padding=8)
        ttk.Label(self, text='기록 시작 없이 실시간 확인 · 오래된 값은 회색으로 표시',
                  wraplength=480).pack(fill='x')
        ttk.Label(self, text='거리 표시는 센서 원점 기준입니다. 범퍼 기준 실측 입력은 보정 탭에서 합니다.',
                  wraplength=480).pack(fill='x')
        self.tabs = ttk.Notebook(self)
        self.tabs.pack(fill='both', expand=True, pady=6)
        overview, lidar, raw = (ttk.Frame(self.tabs) for _ in range(3))
        for tab, title in ((overview, '전체 센서'), (lidar, 'LiDAR 화면'), (raw, '원본 상세')):
            self.tabs.add(tab, text=title)
        canvas = tk.Canvas(overview, highlightthickness=0)
        scroll = ttk.Scrollbar(overview, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        inner = ttk.Frame(canvas)
        item = canvas.create_window(0, 0, window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfigure(item, width=e.width))
        canvas.bind('<Button-4>', lambda e: canvas.yview_scroll(-3, 'units'))
        canvas.bind('<Button-5>', lambda e: canvas.yview_scroll(3, 'units'))
        self.labels = {}
        for title, topic, _ in MONITOR_TOPICS:
            box = ttk.LabelFrame(inner, text=title, padding=8)
            box.pack(fill='x', pady=4, padx=3)
            status = ttk.Label(box, text=topic, wraplength=440, foreground='#666666')
            status.pack(fill='x')
            value = ttk.Label(box, text='미수신', wraplength=440)
            value.pack(fill='x', pady=(5, 0))
            self.labels[topic] = (status, value)
        self.scan_status = ttk.Label(lidar, text='LiDAR 미수신', wraplength=470)
        self.scan_status.pack(fill='x')
        bar = ttk.Frame(lidar)
        bar.pack(fill='x', pady=6)
        ttk.Label(bar, text='표시 반경 (m)').pack(side='left')
        self.radius = tk.StringVar(value='3')
        ttk.Spinbox(bar, from_=.2, to=20., increment=.2, textvariable=self.radius,
                    width=8).pack(side='left', padx=8)
        self.scan_canvas = tk.Canvas(lidar, width=460, height=250, background='#142028',
                                     highlightthickness=0)
        self.scan_canvas.pack(fill='both', expand=True)
        ttk.Label(lidar, text='위: +X, 왼쪽: +Y · 적색 십자: 좌표 원점 · 파랑: 유효 거리\n'
                  '모델 수신 시 base_link 좌표, 없으면 센서 좌표. 화면 밖 점은 생략합니다.\n'
                  '전체 레이·각도·intensities는 원본 상세에서 /scan 선택',
                  wraplength=470).pack(fill='x', pady=5)
        topics = [topic for _, topic, _ in MONITOR_TOPICS
                  if topic not in ('/camera/image_raw', '/camera/image_raw/compressed')]
        topics.append('/imu_native')
        self.raw_topic = tk.StringVar(value='/imu_raw')
        ttk.Combobox(raw, textvariable=self.raw_topic, values=topics, state='readonly',
                     width=40).pack(fill='x')
        ttk.Label(raw, text='ROS 원본 전체 필드 · 단위는 전체 센서 탭 참고 · 약 1초마다 갱신',
                  wraplength=470).pack(fill='x', pady=4)
        ttk.Label(raw, text='gyro 단위: imu_raw는 rad/s · imu_native는 deg/s',
                  wraplength=470).pack(fill='x')
        self.raw = ScrolledText(raw, height=18, wrap='none', state='disabled')
        self.raw.pack(fill='both', expand=True)
        self.raw_key = None
        self.raw_updated = -math.inf

    def update_values(self, snapshot, geometry=None):
        now = time.monotonic()
        data = readings(snapshot, now)
        for topic, (_, status, value, fresh) in data.items():
            labels = self.labels[topic]
            labels[0].configure(text=topic+'\n'+status)
            labels[1].configure(text=value, foreground='#172b3a' if fresh else '#777777')
        messages = snapshot[0]
        scan = messages.get('/scan')
        if str(self.tabs.select()) == str(self.tabs.tabs()[1]):
            self.draw_scan(scan, geometry)
            if scan is not None:
                self.scan_status.configure(text=data['/scan'][1]+'\n'+summary('/scan', scan))
        if str(self.tabs.select()) == str(self.tabs.tabs()[2]):
            topic = self.raw_topic.get()
            if topic != self.raw_key or now-self.raw_updated >= 1:
                msg = messages.get(topic)
                text = message_to_yaml(msg) if msg is not None else '미수신: '+topic
                top = self.raw.yview()[0]
                self.raw.configure(state='normal')
                self.raw.delete('1.0', 'end')
                self.raw.insert('1.0', text)
                self.raw.yview_moveto(top)
                self.raw.configure(state='disabled')
                self.raw_key, self.raw_updated = topic, now

    def draw_scan(self, scan, geometry):
        canvas = self.scan_canvas
        canvas.delete('all')
        width, height = max(100, canvas.winfo_width()), max(100, canvas.winfo_height())
        cx, cy = width/2, height/2
        try:
            radius = float(self.radius.get())
            if not math.isfinite(radius) or not .2 <= radius <= 20:
                raise ValueError()
        except ValueError:
            canvas.create_text(cx, cy, text='표시 반경은 0.2~20 m', fill='white')
            return
        scale = (min(width, height)/2-25)/radius
        for fraction in (.25, .5, .75, 1.):
            r = radius*fraction*scale
            canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline='#3d535e')
            canvas.create_text(cx+4, cy-r+8, text=f'{radius*fraction:g}m', fill='#b2c5ce', anchor='w')
        canvas.create_line(cx, height-10, cx, 10, arrow='last', fill='#c6d9dc')
        canvas.create_line(width-10, cy, 10, cy, arrow='last', fill='#c6d9dc')
        canvas.create_text(cx+15, 12, text='+X', fill='white')
        canvas.create_text(17, cy-12, text='+Y', fill='white')
        frame = '센서 좌표'
        if scan is not None:
            points, frame = scan_points(scan, geometry, radius)
            for x, y in points:
                if x*x+y*y > radius*radius:
                    continue
                px, py = cx-y*scale, cy-x*scale
                canvas.create_oval(px-1, py-1, px+1, py+1, fill='#54c5f8', outline='')
        canvas.create_line(cx-5, cy, cx+5, cy, fill='#ff766c', width=2)
        canvas.create_line(cx, cy-5, cx, cy+5, fill='#ff766c', width=2)
        canvas.create_text(8, height-12, text=frame, fill='white', anchor='w')
