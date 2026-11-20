"""ROS-only guided calibration desktop; no device connection or Jupyter dependency."""

import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import cv2
import numpy as np
from PIL import Image, ImageTk
import rclpy
import yaml

from .suite_model import finite, GUIDES, scalar_report, Session, stats, wheel_fit
from .suite_ros import Acquisition
from .board_geometry import Board


class App:
    def __init__(self, root, node):
        self.root, self.node = root, node
        self.session = None
        self.events = queue.Queue()
        self.busy = False
        root.title('Pinky Pro · 캘리브레이션 스튜디오')
        root.geometry('1180x850')
        self.environment = tk.StringVar(value='rehearsal')
        self.robot = tk.StringVar(value='pinky-pro')
        self.output = tk.StringVar(value=str(Path.cwd()/'calibration_results'))
        bar = ttk.Frame(root, padding=10)
        bar.pack(fill='x')
        ttk.Label(bar, text='이미 launch된 ROS 장치 사용 · 별도 연결 없음').pack(side='left')
        ttk.Combobox(bar, textvariable=self.environment, values=['rehearsal', 'real'],
                     width=12, state='readonly').pack(side='left', padx=8)
        ttk.Entry(bar, textvariable=self.robot, width=18).pack(side='left')
        ttk.Button(bar, text='저장 폴더', command=self.choose).pack(side='left', padx=8)
        ttk.Button(bar, text='새 실험 / 기록 시작', command=lambda: self.job(self.new)).pack(side='left')
        ttk.Button(bar, text='기록 종료', command=lambda: self.job(self.finish)).pack(side='left')
        tk.Button(bar, text='■ 즉시 정지', bg='#c0392b', fg='white',
                  command=lambda: node.stop()).pack(side='right')
        self.status = ttk.Label(root, text='ROS 토픽을 기다리는 중…', padding=10)
        self.status.pack(fill='x')
        body = ttk.Panedwindow(root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10)
        left, right = ttk.Frame(body), ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=1)
        self.preview = ttk.Label(left, text='카메라 미수신')
        self.preview.pack(fill='both', expand=True)
        self.undistort = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text='왜곡 제거 미리보기 (학습 후)', variable=self.undistort).pack()
        ttk.Label(left, text='학습/검증 사진은 버튼을 누른 시점의 새 영상으로 저장합니다.\n'
                  '새 세션과 함께 원본 Bag·CSV가 기록됩니다.', padding=10).pack()
        book = ttk.Notebook(right)
        book.pack(fill='both', expand=True)
        self.fields = {}
        for key, (title, guide) in GUIDES.items():
            tab = ttk.Frame(book, padding=12)
            book.add(tab, text={'camera':'카메라','wheels':'바퀴','lidar':'LiDAR',
                               'ultrasonic':'초음파','imu':'IMU','timing':'주기','extrinsics':'장착'}[key])
            ttk.Label(tab, text=guide, wraplength=490, justify='left').pack(fill='x', pady=12)
            if key == 'camera':
                self.field(tab, 'columns', '내부 코너 가로 수 (칸 수 아님)', '9')
                self.field(tab, 'rows', '내부 코너 세로 수', '6')
                self.field(tab, 'square', '한 칸 실측 길이 (mm)', '30')
                for label, action in [('학습 사진 촬영', lambda: self.capture('training')),
                                      ('학습 계산', self.fit),
                                      ('독립 검증 사진 촬영', lambda: self.capture('validation')),
                                      ('독립 검증 계산', self.validate)]:
                    self.button(tab, label, action)
            elif key == 'wheels':
                self.armed = tk.BooleanVar(value=False)
                ttk.Checkbutton(tab, text='주변을 비우고 바닥에 배치했습니다. 주행 허용',
                                variable=self.armed).pack(anchor='w')
                for label, kind in [('전진 1m', 'forward'), ('후진 1m', 'backward'),
                                    ('좌회전 360°', 'left'), ('우회전 360°', 'right')]:
                    self.button(tab, label, lambda k=kind: self.move(k))
                self.field(tab, 'distance', '실제 중심 이동 거리 (m, 부호 포함)', '')
                self.field(tab, 'angle', '실제 회전각 (deg, 부호 포함)', '')
                self.button(tab, '실측 반복 실험 저장', self.trial)
                self.button(tab, '반복 실험으로 바퀴 추정', self.wheels)
            elif key in ('lidar', 'ultrasonic'):
                self.field(tab, key+'_reference', '줄자로 잰 기준 거리 (m)', '')
                self.button(tab, '10초 정지 수집 → 분석', lambda k=key: self.range_test(k))
            elif key == 'imu':
                self.button(tab, '10초 정지 수집 → 바이어스·노이즈 분석', self.imu_test)
            elif key == 'timing':
                self.sync_label = ttk.Label(tab, text='실물 모드 선택 시 자동 동기화 확인',
                                            wraplength=480)
                self.sync_label.pack(fill='x')
                self.button(tab, '10초 주기·지터·헤더 나이 수집', self.timing)
            elif key == 'extrinsics':
                for key2, label in [('board_x', '바닥 보드 0번 X (base_link 기준 m)'),
                                    ('board_y', '바닥 보드 0번 Y (m)'),
                                    ('board_yaw', '보드 열(+X) 방향 yaw (deg)'),
                                    ('base_height', '바닥에서 base_link 높이 (m)')]:
                    self.field(tab, key2, label, '')
                ttk.Label(tab, text='위 4개를 비우면 높이·보드 기준 각도만 계산합니다.',
                          wraplength=480).pack()
                self.board_confirm = tk.BooleanVar(value=False)
                ttk.Checkbutton(tab, text='빨간 0번·X/Y 방향을 바닥 표시와 확인했습니다.',
                                variable=self.board_confirm).pack(anchor='w')
                self.button(tab, '바닥 보드 촬영 → 높이·각도 추정', self.pose)
        self.log = ScrolledText(root, height=10, wrap='word')
        self.log.pack(fill='x', padx=10, pady=8)
        footer = ttk.Frame(root, padding=8)
        footer.pack(fill='x')
        self.confirm = tk.BooleanVar(value=False)
        ttk.Checkbutton(footer, text='별도 실험·왜곡 제거 영상·TF 등 해당 결과를 직접 확인했습니다.',
                        variable=self.confirm).pack(side='left')
        ttk.Button(footer, text='결과·프로필 저장', command=lambda: self.job(self.export)).pack(side='right')
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh()

    def choose(self):
        value = filedialog.askdirectory()
        if value:
            self.output.set(value)

    def field(self, tab, key, label, default):
        row = ttk.Frame(tab)
        row.pack(fill='x', pady=4)
        ttk.Label(row, text=label).pack(side='left')
        self.fields[key] = tk.StringVar(value=default)
        ttk.Entry(row, textvariable=self.fields[key], width=15).pack(side='right')

    def button(self, tab, label, action):
        ttk.Button(tab, text=label, command=lambda: self.job(action)).pack(fill='x', pady=4)

    def job(self, action):
        if self.busy:
            messagebox.showinfo('실험 진행 중', '현재 실험이 끝날 때까지 기다리세요. 정지는 항상 가능합니다.')
            return
        # Read all Tk variables on the Tk thread only.
        self.inputs = {k: v.get() for k, v in self.fields.items()}
        self.options = dict(output=self.output.get(), environment=self.environment.get(),
                            robot=self.robot.get(), confirm=self.confirm.get(),
                            armed=self.armed.get(), board_confirm=self.board_confirm.get())
        self.busy = True

        def run():
            try:
                self.events.put(('ok', action()))
            except Exception as error:
                self.events.put(('error', str(error)))
            finally:
                self.events.put(('done', None))
        threading.Thread(target=run, daemon=True).start()

    def require(self):
        if not self.session or not self.node.writer:
            raise ValueError('새 실험을 시작하세요. 종료된 세션은 새로 만들어야 합니다.')
        return self.session

    def new(self):
        if self.node.writer:
            raise ValueError('기존 기록을 먼저 종료하세요.')
        self.session = Session(self.options['output'], self.options['environment'], self.options['robot'])
        self.configure_board()
        self.node.begin(self.session)
        return f'기록 시작: {self.session.root}'

    def finish(self):
        self.node.end()
        return '기록 종료. 프로필 저장은 가능합니다. 추가 실험에는 새 세션을 만드세요.'

    def capture(self, purpose):
        self.configure_board()
        count = self.require().capture(self.node.frame(), purpose)
        return f'{purpose}: {count}장 저장. 동일 자세 반복보다 위치·거리·기울기를 바꾸세요.'

    def fit(self):
        self.configure_board()
        return self.require().camera_fit(self.inputs['square'])

    def validate(self):
        return self.require().camera_validate()

    def move(self, kind):
        self.require()
        if not self.options['armed']:
            raise ValueError('주변 안전 확인 후 주행 허용을 선택하세요.')
        self.node.start_motion(kind)
        return '주행 중. 정지 후 실측값을 입력하세요. 제한 시간 60초 / 0.04m/s / 0.15rad/s'

    def trial(self):
        session = self.require()
        with self.node.lock:
            t = dict(self.node.last_motion or {})
        if not t.get('complete'):
            raise ValueError('정상 목표 도달한 실험만 저장합니다. 중단 실험은 다시 실행하세요.')
        t.update(distance_m=finite(self.inputs['distance']),
                 angle_rad=math.radians(finite(self.inputs['angle'])))
        session.trials.append(t)
        self.node.last_motion = None
        session.write('wheel_trials.yaml', session.trials)
        return f'{len(session.trials)}회 저장. 반복 전후진·좌우 회전을 모두 포함하세요.'

    def wheels(self):
        session = self.require()
        result = wheel_fit(session.trials)
        session.results['wheels'] = result
        return result

    def collect(self, topic):
        self.require()
        start = time.monotonic()
        time.sleep(10)
        with self.node.lock:
            values = [v for v in self.node.values[topic] if v[0] >= start]
        if len(values) < 10 or not self.node.fresh(topic):
            raise ValueError(f'{topic}: 데이터 부족 또는 수신 중단. launch와 센서 옵션을 확인하세요.')
        return values

    def range_test(self, key):
        reference = finite(self.inputs[key+'_reference'])
        if reference <= 0:
            raise ValueError('거리 기준은 양수 m입니다.')
        topic = '/scan' if key == 'lidar' else '/us_sensor/range'
        values = self.collect(topic)
        report = scalar_report([v[2]['distance'] for v in values], reference)
        self.session.results.setdefault(key, []).append(report)
        self.session.write('results.yaml', self.session.results)
        return report

    def imu_test(self):
        values = self.collect('/imu_raw')
        report = {key: stats([v[2][key] for v in values]) for key in values[0][2]
                  if key not in ('header_age_s', 'clock_sync_verified')}
        if (max(report['angular_velocity_'+a]['sample_std'] or 0 for a in 'xyz') > .02
                or max(abs(report['angular_velocity_'+a]['mean']) for a in 'xyz') > .05):
            raise ValueError('정지 중 gyro 변동이 큽니다. 로봇을 멈추고 재수집하세요.')
        self.session.results['imu_stationary'] = report
        return report

    def timing(self):
        self.require()
        start = time.monotonic()
        time.sleep(10)
        report = {}
        with self.node.lock:
            for topic, series in self.node.values.items():
                series = [v for v in series if v[0] >= start]
                if len(series) > 2:
                    report[topic] = {'receive_interval_s': stats(np.diff([v[0] for v in series])),
                                     'delay_status': 'not_measured: source exposure/clock reference required'}
                    if self.node.synchronized() and self.session.meta['environment'] == 'real':
                        ages = [v[2]['header_age_s'] for v in series
                                if 'header_age_s' in v[2] and v[2].get('clock_sync_verified')]
                        if ages:
                            report[topic]['header_to_receive_s'] = stats(ages)
                            report[topic]['delay_status'] = 'header age only; excludes exposure/driver time'
        if not report:
            raise ValueError('수신 데이터가 없습니다.')
        self.session.results['timing'] = report
        self.session.results['time_sync'] = self.node.sync_state.copy()
        return report

    def pose(self):
        session = self.require()
        if not self.options['board_confirm']:
            raise ValueError('0번 코너와 보드 X/Y 방향을 먼저 확인하세요.')
        self.configure_board()
        values = [self.inputs[key].strip() for key in
                  ('board_x', 'board_y', 'board_yaw', 'base_height')]
        registration = None
        if any(values):
            if not all(values):
                raise ValueError('등록 기준 4개를 모두 입력하거나 모두 비우세요.')
            registration = [finite(v) for v in values]
            registration[2] = math.radians(registration[2])
        return session.floor_capture(self.node.frame(), registration)

    def configure_board(self):
        if not self.session:
            raise ValueError('먼저 새 실험을 시작하세요.')
        self.session.configure_board(Board(int(self.inputs['columns']), int(self.inputs['rows']),
                                           finite(self.inputs['square'])))

    def export(self):
        if not self.session:
            raise ValueError('먼저 실험을 시작하세요.')
        self.node.stop('결과 저장')
        path = self.session.export(self.options['confirm'])
        self.node.end()
        return f'프로필 저장: {path}\ncalibration_profile:={path} 인자로 launch를 다시 실행하세요.'

    def refresh(self):
        self.node.sync_enabled = self.environment.get() == 'real'
        self.sync_label.configure(text=self.node.sync_state.get('reason', '동기화 대기')
                                  if self.node.sync_enabled else '리허설: 장치 간 동기화 불필요')
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == 'done':
                self.busy = False
            else:
                text = value if isinstance(value, str) else yaml.safe_dump(value, allow_unicode=True)
                self.log.insert('end', ('오류: ' if kind == 'error' else '')+text+'\n')
                self.log.see('end')
        with self.node.lock:
            states = ['✓ '+t if self.node.fresh(t) else '— '+t for t in self.node.latest]
            motion = '주행 중' if self.node.motion else (self.node.last_motion or {}).get('reason','정지')
        self.status.configure(text=f'{motion} · '+(' | '.join(states) or 'ROS 토픽 미수신'))
        try:
            frame = self.node.frame()
            if self.undistort.get() and self.session and self.session.camera:
                k,d,_,_ = self.session.camera
                frame = cv2.undistort(frame, k, d)
            if self.session:
                board = self.session.board
                corners = board.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if corners is not None:
                    cv2.drawChessboardCorners(frame, board.pattern, corners, True)
                    for index, label in [(0, '0'), (board.columns-1, '+X'),
                                          ((board.rows-1)*board.columns, '+Y')]:
                        p = tuple(corners[index, 0].astype(int))
                        cv2.putText(frame, label, p, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            image.thumbnail((550, 410))
            self.photo = ImageTk.PhotoImage(image)
            self.preview.configure(image=self.photo)
        except ValueError:
            self.preview.configure(image='', text='카메라 미수신 · launch/ROS 설정을 확인하세요.')
        self.root.after(150, self.refresh)

    def close(self):
        self.node.stop('UI 종료')
        if self.busy:
            messagebox.showinfo('실험 마무리', '주행을 정지했습니다. 계산/수집이 끝나면 창을 닫으세요.')
            return
        self.node.close()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Acquisition()
    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        if node.running:
            node.close()
        rclpy.try_shutdown()
