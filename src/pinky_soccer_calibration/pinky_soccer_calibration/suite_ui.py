"""ROS-only guided calibration desktop; no device connection or Jupyter dependency."""

import math
from pathlib import Path
import queue
import sys
import threading
import time
import uuid
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import cv2
import numpy as np
from PIL import Image, ImageTk
import rclpy
import yaml

from .suite_model import finite, GUIDES, scalar_report, Session, stats
from .suite_ros import Acquisition, MOTION_LINEAR_SPEED, MOTION_ANGULAR_SPEED
from .board_geometry import Board
from .board_preview import annotate_board
from .monitor_view import SensorMonitor
from .wheel_measurement import distance_measurement, rotation_measurement
from .project_store import utc
from .project_view import ProjectView
from .camera_view import CameraCalibrationView
from .ball_review_view import BallReviewView
from .lidar_field import save_field


class App:
    def __init__(self, root, node):
        self.root, self.node = root, node
        self.webcam = getattr(node, 'source_kind', '') == 'webcam'
        self.session = None
        self.events = queue.Queue()
        self.busy = False
        root.title('Pinky Pro · 캘리브레이션 스튜디오')
        root.geometry('1180x850')
        self.environment = tk.StringVar(value='rehearsal')
        self.robot = tk.StringVar(value='webcam:'+node.device if self.webcam else 'pinky-pro')
        self.output = tk.StringVar(value=str(Path.cwd()/'calibration_results'))
        bar = ttk.Frame(root, padding=10)
        bar.pack(fill='x')
        ttk.Label(bar, text=('웹캠 전용 리허설 · 로봇 연결/주행 없음' if self.webcam else
                            '이미 launch된 ROS 장치 사용 · 별도 연결 없음')).pack(side='left')
        ttk.Combobox(bar, textvariable=self.environment, values=['rehearsal', 'real'],
                     width=12, state='disabled' if self.webcam else 'readonly').pack(side='left', padx=8)
        ttk.Entry(bar, textvariable=self.robot, width=18,
                  state='disabled' if self.webcam else 'normal').pack(side='left')
        actions = ttk.Frame(root, padding=4)
        actions.pack(fill='x')
        for title, callback in [('새 프로젝트', self.create_project), ('저장 폴더 불러오기', self.open_project),
                                ('프로젝트 저장', lambda: self.job(self.save_project)),
                                ('기록 시작/재개', lambda: self.job(self.start_recording)),
                                ('기록 종료', lambda: self.job(self.finish))]:
            ttk.Button(actions, text=title, command=callback).pack(side='left', padx=3)
        tk.Button(bar, text='웹캠 전용 · 주행 없음' if self.webcam else '■ 즉시 정지',
                  bg='#808080' if self.webcam else '#c0392b', fg='white',
                  state='disabled' if self.webcam else 'normal',
                  command=lambda: node.stop()).pack(side='right')
        self.status = ttk.Label(root, text='ROS 토픽을 기다리는 중…', padding=10,
                                wraplength=1100, justify='left')
        self.status.pack(fill='x')
        body = ttk.Panedwindow(root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10)
        left, right = ttk.Frame(body), ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=1)
        self.preview = ttk.Label(left, text='카메라 미수신')
        self.preview.pack(fill='both', expand=True)
        self.board_status = ttk.Label(left, text='체커보드 인식: 카메라 수신 대기',
                                      wraplength=530, padding=6)
        self.board_status.pack(fill='x')
        self.undistort = tk.BooleanVar(value=False)
        self.undistort_toggle = ttk.Checkbutton(left, text='왜곡 제거 미리보기 (보정 계산 후)', variable=self.undistort)
        self.undistort_toggle.pack()
        ttk.Label(left, text=('웹캠 로컬 미리보기·체커: JPEG\n'
                  '촬영 버튼: 새로 읽은 BGR 원본 1장 → PNG·CameraInfo·Bag·CSV 보존' if self.webcam else
                  '미리보기·체커 표시: JPEG 압축 영상\n'
                  '촬영 버튼: 서비스로 새 원본 1장 → PNG·CameraInfo·Bag·CSV 보존'),
                  padding=10).pack()
        self.capture_status = ttk.Label(left, text='원본 촬영: 대기', wraplength=530, padding=6)
        self.capture_status.pack(fill='x')
        book = ttk.Notebook(right)
        book.pack(fill='both', expand=True)
        self.monitor = SensorMonitor(book)
        book.add(self.monitor, text='센서 모니터')
        self.ball_review = BallReviewView(book, lambda: self.project_view.refresh(),
                                        lambda: self.busy)
        self.ball_review.configure_source(node)
        book.add(self.ball_review, text='공 인식 검토')
        self.project_view = ProjectView(book, self)
        book.add(self.project_view, text='측정 기록')
        self.book = book
        self.fields = {}
        self.range_geometry_label = {}
        for key, (title, guide) in GUIDES.items():
            tab = ttk.Frame(book, padding=12)
            book.add(tab, text={'camera':'카메라 보정','wheels':'바퀴','lidar':'LiDAR',
                               'ultrasonic':'초음파','imu':'IMU','timing':'주기','extrinsics':'장착'}[key])
            if self.webcam and key in ('wheels', 'lidar', 'ultrasonic', 'imu'):
                book.tab(tab, state='disabled')
            if self.webcam and key == 'extrinsics':
                guide = ('웹캠 내부 보정·독립 검증 후 보드를 바닥에 평평하게 놓고 웹캠을 고정하세요. '
                         '높이·roll/pitch를 추정합니다. 5회 이상 반복 권장. 카메라 위치/노트북 화면 각도를 '
                         '바꾸면 다시 측정하세요. 공 거리 원점은 웹캠의 바닥 투영점입니다.')
            if key != 'camera':
                ttk.Label(tab, text=guide, wraplength=490, justify='left').pack(fill='x', pady=12)
            if key == 'camera':
                self.camera_tab = tab
                self.camera_view = CameraCalibrationView(tab, self)
                self.camera_view.pack(fill='both', expand=True)
            elif key == 'wheels':
                self.armed = tk.BooleanVar(value=False)
                ttk.Checkbutton(tab, text='주변을 비우고 바닥에 배치했습니다. 주행 허용',
                                variable=self.armed).pack(anchor='w')
                ttk.Label(tab, text='① 전후진 실험 · 범퍼 중앙의 시작~끝 직선 거리 측정').pack(fill='x')
                for label, kind in [('전진 1m', 'forward'), ('후진 1m', 'backward')]:
                    self.button(tab, label, lambda k=kind: self.move(k))
                self.field(tab, 'distance', '범퍼 중앙 시작~끝 거리 (m, 전진+/후진-)', '')
                self.button(tab, '전후진 실측 저장', lambda: self.trial('straight'))
                ttk.Label(tab, text='② 회전 실험 · 한 바퀴 누적 각도 측정').pack(fill='x')
                for label, kind in [('좌회전 360°', 'left'), ('우회전 360°', 'right')]:
                    self.button(tab, label, lambda k=kind: self.move(k))
                self.field(tab, 'angle', '실제 누적 회전각 (deg, 좌+/우-)', '')
                self.button(tab, '회전 실측 저장', lambda: self.trial('rotation'))
                self.button(tab, '반복 실험으로 바퀴 추정', self.wheels)
            elif key in ('lidar', 'ultrasonic'):
                self.field(tab, key+'_reference', '범퍼 전면 바깥쪽 중앙~벽 거리 (m)', '')
                self.range_geometry_label[key] = ttk.Label(tab, text='범퍼 모델 수신 대기',
                                                           wraplength=480)
                self.range_geometry_label[key].pack(fill='x', pady=4)
                self.button(tab, '10초 정지 수집 → 분석', lambda k=key: self.range_test(k))
                if key == 'lidar':
                    ttk.Separator(tab).pack(fill='x', pady=8)
                    ttk.Label(tab, text='경기장 윤곽 저장 · 현재 스캔 1장 기준\n'
                              '사람·장애물을 치우고 로봇을 정지하세요. 가려진 벽은 저장되지 않습니다.\n'
                              '지도 원점은 촬영 당시 로봇/센서 위치이며 경기장 중앙·범퍼 기준이 아닙니다.',
                              wraplength=490).pack(fill='x')
                    self.field(tab, 'lidar_map_resolution', '지도 해상도 (m/픽셀)', '0.01')
                    self.lidar_clear = tk.BooleanVar(value=False)
                    ttk.Checkbutton(tab, text='로봇 정지·경기장 안 사람/장애물 제거를 확인했습니다.',
                                    variable=self.lidar_clear).pack(anchor='w')
                    self.button(tab, '현재 LiDAR 경기장 모양 저장', self.save_lidar_field)
                    self.lidar_saved = ttk.Label(tab, text='프로젝트를 만들거나 불러온 뒤 저장하세요.',
                                                 wraplength=490)
                    self.lidar_saved.pack(fill='x')
            elif key == 'imu':
                self.button(tab, '10초 정지 수집 → 바이어스·노이즈 분석', self.imu_test)
            elif key == 'timing':
                self.sync_label = ttk.Label(tab, text='실물 모드 선택 시 자동 동기화 확인',
                                            wraplength=480)
                self.sync_label.pack(fill='x')
                self.button(tab, '10초 주기·지터·헤더 나이 수집', self.timing)
            elif key == 'extrinsics':
                self.mount_tab = tab
                ttk.Label(tab, text='장착 확인용 체커보드 · 내부 보정용과 다른 보드 사용 가능',
                          wraplength=490).pack(fill='x')
                self.field(tab, 'floor_columns', '장착용 내부 코너 가로 수', '9')
                self.field(tab, 'floor_rows', '장착용 내부 코너 세로 수', '6')
                self.field(tab, 'floor_square', '장착용 한 칸 실측 길이 (mm)', '30')
                self.button(tab, '장착 보드 설정 저장', self.configure_floor_board)
                self.floor_confirm = tk.BooleanVar(value=False)
                ttk.Checkbutton(tab, text=('보드를 바닥에 평평하게 놓고 웹캠을 고정했습니다.' if self.webcam else
                                          '보드를 바닥에 평평하게 놓고 로봇을 수평·정지시켰습니다.'),
                                variable=self.floor_confirm).pack(anchor='w')
                self.button(tab, '바닥 보드 촬영 → 높이·각도 추정', self.pose)
                self.mount_status = ttk.Label(tab, text='바닥 높이·좌우 기울기·상하 기울기: 촬영 대기',
                                              wraplength=480, padding=8)
                self.mount_status.pack(fill='x')
        self.log = ScrolledText(root, height=6, wrap='word')
        self.log.pack(side='bottom', before=body, fill='x', padx=10, pady=8)
        footer = ttk.Frame(root, padding=8)
        footer.pack(side='bottom', before=self.log, fill='x')
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

    def create_project(self):
        if self.busy:
            return
        value = filedialog.askdirectory(title='새 프로젝트를 만들 상위 폴더')
        if value:
            self.output.set(value)
            self.job(self.new)

    def open_project(self):
        if self.busy:
            return
        path = filedialog.askdirectory(title='보정 프로젝트 폴더 선택')
        if path:
            self.job(lambda: self.load_project(path))

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
        if self.node.motion and getattr(action, '__name__', '') != 'finish':
            messagebox.showinfo('주행 중', '주행을 완료하거나 즉시 정지한 뒤 기록을 편집하세요.')
            return
        # Read all Tk variables on the Tk thread only.
        self.inputs = {k: v.get() for k, v in self.fields.items()}
        self.options = dict(output=self.output.get(), environment=self.environment.get(),
                            robot=self.robot.get(), confirm=self.confirm.get(),
                            armed=self.armed.get(), floor_confirm=self.floor_confirm.get())
        self.options['lidar_clear'] = self.lidar_clear.get()
        self.busy = True

        def run():
            try:
                result = action()
                if result is not None:
                    self.events.put(('ok', result))
            except Exception as error:
                self.events.put(('error', str(error)))
            finally:
                self.events.put(('done', None))
        threading.Thread(target=run, daemon=True).start()

    def require(self):
        if not self.session:
            raise ValueError('프로젝트를 만들거나 저장 폴더를 불러오세요.')
        return self.session

    def save_lidar_field(self):
        """Save a stationary reference without starting recording or changing calibration values."""
        session = self.require()
        if self.webcam:
            raise ValueError('웹캠 전용 모드에서는 LiDAR를 저장할 수 없습니다.')
        if not self.options['lidar_clear']:
            raise ValueError('로봇 정지·경기장 안 사람/장애물 제거 확인을 체크하세요.')
        messages, seen, _ = self.node.monitor_snapshot()
        scan = messages.get('/scan')
        if scan is None:
            raise ValueError('/scan 미수신: 로봇/시뮬레이터 launch와 ROS 연결을 확인하세요.')
        try:
            geometry = self.node.measurement_geometry()
        except ValueError:
            geometry = None
        now = time.monotonic()
        directory, info = save_field(
            session.root, scan, received_monotonic=seen.get('/scan', -math.inf),
            received_ros_ns=self.node.get_clock().now().nanoseconds, geometry=geometry,
            odom=messages.get('/odom'),
            odom_age_s=now-seen['/odom'] if '/odom' in seen else None,
            environment=session.meta['environment'], robot_id=session.meta['robot_id'],
            resolution=float(self.inputs['lidar_map_resolution']))
        return (f'LiDAR 경기장 관측 저장: {directory}\n'
                f'지도: {directory / "map.yaml"}\n'
                f'벽 관측 {info["obstacle_returns"]}개 · 좌표 {info["capture_coordinate_frame"]} · '
                '원본 Bag/CSV 보존. 보정 프로필과 별개이며 자동 적용되지 않습니다.')

    def require_recording(self):
        session = self.require()
        if not self.node.writer:
            raise ValueError('촬영/수집/주행 전에 기록 시작/재개를 누르세요.')
        return session

    def can_switch(self):
        if self.node.writer or self.node.motion:
            raise ValueError('프로젝트를 바꾸기 전에 기록을 종료하세요.')
        if self.node.last_motion and self.node.last_motion.get('complete'):
            raise ValueError('직전 주행 실측값을 먼저 저장하세요.')

    def bind_source(self, session, creating=False):
        if getattr(self.node, 'source_kind', '') == 'webcam':
            self.node.bind_project(session, creating=creating)
        elif session.meta.get('image_source', {}).get('kind') == 'webcam':
            raise ValueError('웹캠 프로젝트는 -webcam 옵션으로 실행한 UI에서만 열 수 있습니다.')

    def new(self):
        self.can_switch()
        board = Board(int(self.inputs['columns']), int(self.inputs['rows']), finite(self.inputs['square']))
        session = Session(self.options['output'], self.options['environment'], self.options['robot'])
        session.configure_board(board)
        try:
            self.bind_source(session, creating=True)
        except Exception:
            session.release()
            raise
        if self.session:
            self.session.save()
            self.session.release()
        self.session = session
        self.events.put(('project_loaded', self.session.meta.copy()))
        return f'새 프로젝트: {self.session.root}. 촬영/실험 전에 기록 시작/재개를 누르세요.'

    def load_project(self, path):
        self.can_switch()
        if self.session and Path(path).resolve() == self.session.root:
            return '현재 열려 있는 프로젝트입니다.'
        session = Session.open(path)
        try:
            self.bind_source(session)
        except Exception:
            session.release()
            raise
        if self.session:
            self.session.save()
            self.session.release()
        self.session = session
        self.node.last_motion = session.meta.get('pending_motion')
        self.events.put(('project_loaded', session.meta.copy()))
        return f'프로젝트 불러옴: {session.root}. 과거 기록 검토/재계산 또는 기록 재개가 가능합니다.'

    def save_project(self):
        self.configure_board()
        return str(self.require().save())

    def start_recording(self):
        session = self.require()
        if self.options['environment'] != session.meta['environment'] or self.options['robot'] != session.meta['robot_id']:
            raise ValueError('프로젝트 환경/로봇 식별명과 화면 설정이 다릅니다. 다른 로봇은 새 프로젝트를 만드세요.')
        self.node.begin(session)
        return '기록 재개: 원본 Bag/CSV는 새 파일로 보존됩니다.'

    def finish(self):
        self.node.end()
        if self.session:
            self.session.save()
        return '기록 종료·프로젝트 저장 완료. 검토/재계산 가능하며 기록 시작/재개로 이어갈 수 있습니다.'

    def capture(self, purpose):
        self.configure_board()
        frame, metadata = self.capture_source()
        count = self.require().capture(frame, purpose, provenance=metadata)
        return f'{purpose}: 원본 {count}장 저장. 동일 자세 반복보다 위치·거리·기울기를 바꾸세요.'

    def capture_source(self):
        """Use the raw snapshot service exclusively; preview pixels are never calibration input."""
        session = self.require_recording()
        self.events.put(('capture_waiting', None))
        try:
            sample = self.node.capture_raw()
            sample.metadata = session.save_raw_capture(sample.frame, sample.metadata)
            self.node.record_capture(sample, session)
        except Exception as error:
            self.events.put(('capture_error', str(error)))
            raise
        self.events.put(('capture', sample.metadata))
        return sample.frame, sample.metadata

    def fit(self):
        self.configure_board()
        return self.require().camera_fit(self.inputs['square'])

    def validate(self):
        self.configure_board()
        return self.require().recalculate('camera')

    def move(self, kind):
        self.require_recording()
        if not self.options['armed']:
            raise ValueError('주변 안전 확인 후 주행 허용을 선택하세요.')
        with self.node.lock:
            if self.node.last_motion and self.node.last_motion.get('complete'):
                raise ValueError('직전 실험의 실측값을 먼저 저장하세요.')
            geometry = self.node.measurement_geometry() if kind in ('forward', 'backward') else None
            reference = geometry.wheel_reference() if geometry else None
            if reference:
                model = self.session.root / ('robot_description_'+geometry.sha256+'.urdf')
                if not model.exists():
                    model.write_text(geometry.description)
            self.node.start_motion(kind)
            if reference:
                self.node.motion['measurement_geometry'] = reference
        return (f'주행 중. 정지 후 실측값을 입력하세요. 제한 시간 60초 / '
                f'{MOTION_LINEAR_SPEED:.2f}m/s / {MOTION_ANGULAR_SPEED:.2f}rad/s')

    def trial(self, experiment):
        session = self.require()
        with self.node.lock:
            t = dict(self.node.last_motion or {})
        if not t.get('complete'):
            raise ValueError('정상 목표 도달한 실험만 저장합니다. 중단 실험은 다시 실행하세요.')
        straight = t.get('kind') in ('forward', 'backward')
        if experiment != ('straight' if straight else 'rotation'):
            raise ValueError('직전 주행 종류에 맞는 저장 버튼을 누르세요.')
        if straight:
            reference = t['measurement_geometry']
            if self.node.measurement_geometry().sha256 != reference['robot_description_sha256']:
                raise ValueError('주행 후 모델이 바뀌었습니다. 원래 모델을 복원한 뒤 저장하세요.')
            t.update(distance_measurement(t['kind'], finite(self.inputs['distance']), reference))
        else:
            t.update(rotation_measurement(t['kind'], finite(self.inputs['angle'])))
        t.update(id=uuid.uuid4().hex, recorded_utc=utc())
        session.trials.append(t)
        try:
            category = 'straight' if straight else 'rotation'
            types = {'straight_displacement', 'straight_distance'} if straight else {'rotation_angle'}
            session.write('wheel_'+category+'_trials.yaml',
                          [v for v in session.trials if v.get('measurement_type') in types])
            session.write('wheel_trials.yaml', session.trials)
        except Exception:
            session.trials.pop()
            raise
        session.invalidate('wheels')
        session.meta.pop('pending_motion', None)
        session.save()
        with self.node.lock:
            self.node.last_motion = None
        detail = (f"시작~끝 실측 거리 {t['distance_m']:.4f}m · 편향/각도 역산 없음" if straight else
                  f"실측 누적 회전 {t['measured_angle_deg']:.2f}°")
        return f'{len(session.trials)}회 저장: {detail}. 각 방향 3회 이상 반복 후 바퀴 추정.'

    def wheels(self):
        session = self.require()
        return session.recalculate('wheels')

    def collect(self, topic):
        self.require_recording()
        start = time.monotonic()
        time.sleep(10)
        with self.node.lock:
            values = [v for v in self.node.values[topic] if v[0] >= start]
        if len(values) < 10 or not self.node.fresh(topic):
            raise ValueError(f'{topic}: 데이터 부족 또는 수신 중단. launch와 센서 옵션을 확인하세요.')
        return values

    def range_test(self, key):
        distance = finite(self.inputs[key+'_reference'])
        geometry = self.node.measurement_geometry()
        topic = '/scan' if key == 'lidar' else '/us_sensor/range'
        with self.node.lock:
            if not self.node.fresh(topic):
                raise ValueError(f'{topic}: 센서 수신을 먼저 확인하세요.')
            frame = self.node.latest[topic].header.frame_id
        conversion = geometry.reference(distance, frame, key)
        values = self.collect(topic)
        if any(v[2].get('robot_description_sha256') != geometry.sha256
               or v[2].get('sensor_frame') != frame for v in values):
            raise ValueError('수집 중 모델 또는 센서 프레임이 바뀌었습니다. 다시 수집하세요.')
        if self.node.measurement_geometry().sha256 != geometry.sha256:
            raise ValueError('수집 중 모델이 바뀌었습니다. 다시 수집하세요.')
        report = scalar_report([v[2]['distance'] for v in values],
                               conversion['sensor_reference_m'])
        report.update(conversion)
        model_path = self.session.root / ('robot_description_'+geometry.sha256+'.urdf')
        if not model_path.exists():
            model_path.write_text(geometry.description)
        report['robot_description_file'] = model_path.name
        self.session.add_measurement(key, {'samples': [v[2] for v in values],
                                          'reference': dict(conversion, robot_description_file=model_path.name)})
        self.session.recalculate(key)
        return report

    def imu_test(self):
        values = self.collect('/imu_raw')
        self.session.add_measurement('imu_stationary', {'samples': [v[2] for v in values]})
        return self.session.recalculate('imu_stationary')

    def timing(self):
        self.require_recording()
        start = time.monotonic()
        time.sleep(10)
        report = {}
        topics = {}
        with self.node.lock:
            for topic, series in self.node.values.items():
                series = [v for v in series if v[0] >= start]
                if len(series) > 2:
                    topics[topic] = series
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
        self.session.results['time_sync'] = self.node.sync_state.copy()
        self.session.add_measurement('timing', {'topics': topics,
                                              'clock_sync_verified': self.node.synchronized()
                                              and self.session.meta['environment'] == 'real'})
        return self.session.recalculate('timing')

    def pose(self):
        session = self.require()
        if not self.options['floor_confirm']:
            raise ValueError('보드를 바닥에 평평하게 놓고 '+
                             ('웹캠을 고정한 뒤 확인하세요.' if getattr(self, 'webcam', False) else
                              '로봇을 수평·정지시킨 뒤 확인하세요.'))
        self.configure_floor_board()
        frame, metadata = self.capture_source()
        result = session.floor_capture(frame, None, provenance=metadata)
        self.events.put(('mount_result', result))
        return result

    def configure_board(self):
        if not self.session:
            raise ValueError('먼저 새 실험을 시작하세요.')
        self.session.configure_board(Board(int(self.inputs['columns']), int(self.inputs['rows']),
                                           finite(self.inputs['square'])))

    def configure_floor_board(self):
        session = self.require()
        board = Board(int(self.inputs['floor_columns']), int(self.inputs['floor_rows']),
                      finite(self.inputs['floor_square']))
        session.configure_floor_board(board)
        return f'장착용 보드 설정: 내부 코너 {board.pattern}, 한 칸 {board.square_mm:g}mm'

    def export(self):
        if not self.session:
            raise ValueError('먼저 실험을 시작하세요.')
        if self.node.last_motion and self.node.last_motion.get('complete'):
            raise ValueError('완료한 바퀴 주행의 실측값을 먼저 저장하세요.')
        self.node.stop('결과 저장')
        path = self.session.export(self.options['confirm'])
        self.node.end()
        return f'프로필 저장: {path}\ncalibration_profile:={path} 인자로 launch를 다시 실행하세요.'

    def refresh(self):
        geometry = None
        try:
            geometry = self.node.measurement_geometry()
            front = geometry.front
            geometry_text = (f'모델 범퍼 전면: base_link X={front[0]:.4f}m. '
                             '입력은 범퍼에서 벽까지, 계산은 센서 위치로 자동 변환합니다.')
        except ValueError as error:
            geometry_text = str(error)
        self.monitor.update_values(self.node.monitor_snapshot(), geometry)
        if self.session:
            saved = sorted(p for p in (self.session.root/'lidar_fields').glob('*/observation.yaml')
                           if not p.parent.name.startswith('.'))
            self.lidar_saved.configure(text=(f'저장된 경기장 관측 {len(saved)}개\n'
                                            f'최근 지도: {saved[-1].parent / "map.yaml"}' if saved else
                                            '저장된 경기장 관측 없음 · 기록 시작 없이 저장 가능'))
        for label in self.range_geometry_label.values():
            label.configure(text=geometry_text)
        if geometry is not None:
            for key, label in self.range_geometry_label.items():
                try:
                    with self.node.lock:
                        msg = self.node.latest['/scan' if key == 'lidar' else '/us_sensor/range']
                    conversion = geometry.reference(self.fields[key+'_reference'].get(),
                                                    msg.header.frame_id, key)
                    label.configure(text=geometry_text +
                        f"\n변환된 센서 기준: {conversion['sensor_reference_m']:.4f}m")
                except (ValueError, KeyError):
                    pass
        self.node.sync_enabled = self.environment.get() == 'real'
        self.sync_label.configure(text=self.node.sync_state.get('reason', '동기화 대기')
                                  if self.node.sync_enabled else '리허설: 장치 간 동기화 불필요')
        while not self.events.empty():
            kind, value = self.events.get()
            if kind == 'done':
                self.busy = False
                self.project_view.refresh()
                self.camera_view.refresh()
                self.confirm.set(False)
                if self.session:
                    summary = self.session.results.get('camera_mount', {})
                    if all(k in summary for k in ('height_m', 'roll_rad', 'pitch_rad')):
                        self.mount_status.configure(
                            text=(f"현재 기록 {summary.get('repeats', 0)}회 · URDF 높이 사용: "
                                  if summary.get('height_override') else
                                  f"현재 기록 {summary.get('repeats', 0)}회 평균: ") +
                                 f"높이 {summary['height_m']['mean']*1000:.1f}mm · "
                                 f"roll {math.degrees(summary['roll_rad']['mean']):.2f}° · "
                                 f"pitch {math.degrees(summary['pitch_rad']['mean']):.2f}°")
                    elif 'floor' in self.session.stale:
                        self.mount_status.configure(text='장착 결과 재계산 필요 · 측정 기록에서 장착 기록을 선택하세요.')
            elif kind == 'project_loaded':
                self.environment.set(value['environment'])
                self.robot.set(value['robot_id'])
                for key, v in zip(('columns', 'rows', 'square'),
                                  (*self.session.board.pattern, self.session.board.square_mm)):
                    self.fields[key].set(str(v))
                for key, v in zip(('floor_columns', 'floor_rows', 'floor_square'),
                                  (*self.session.floor_board.pattern,
                                   self.session.floor_board.square_mm)):
                    self.fields[key].set(str(v))
                self.book.select(self.camera_tab)
            elif kind == 'capture_waiting':
                self.capture_status.configure(text='원본 촬영 요청 중 · JPEG 미리보기는 계속 표시',
                                              foreground='#a05a00')
            elif kind == 'capture_error':
                self.capture_status.configure(text='원본 촬영 실패: '+value, foreground='#b03030')
            elif kind == 'mount_result':
                self.mount_status.configure(
                    text=f"높이 {value['height_m']*1000:.1f}mm · "
                         f"좌우 기울기(roll) {math.degrees(value['roll_rad']):.2f}° · "
                         f"상하 기울기(pitch) {math.degrees(value['pitch_rad']):.2f}°\n"
                         f"반복 {value['repeats']}회 · 재투영 RMS {value['reprojection_rms_px']:.3f}px\n"
                         '결과는 측정값으로 저장됩니다. 로봇 기준 X/Y/yaw·전체 TF는 생성하지 않습니다.')
            elif kind == 'capture':
                info = value['camera_info']
                stamp = info['header']['stamp']
                self.capture_status.configure(
                    text=f"원본 수신·저장: {value['width']}×{value['height']} · "
                         f"시각 {stamp['sec']}.{stamp['nanosec']:09d}\n"
                         f"{value['raw_image_file']}", foreground='#16733c')
            else:
                text = value if isinstance(value, str) else yaml.safe_dump(value, allow_unicode=True)
                self.log.insert('end', ('오류: ' if kind == 'error' else '')+text+'\n')
                self.log.see('end')
        with self.node.lock:
            states = [('✓ ' if (geometry is not None if t == '/robot_description'
                                else self.node.fresh(t)) else '— ')+t
                      for t in self.node.latest]
            motion = '주행 중' if self.node.motion else (self.node.last_motion or {}).get('reason','정지')
        self.status.configure(text=f'{motion} · '+(' | '.join(states) or 'ROS 토픽 미수신'))
        try:
            self.undistort_toggle.configure(state='disabled' if self.book.select() == str(self.ball_review)
                                            else 'normal')
            if self.book.select() == str(self.ball_review):
                frame = self.ball_review.update_frame(self.node, self.session)
                self.board_status.configure(text='공 인식 검토 · 경기 공통 알고리즘 / JPEG 픽셀 좌표',
                                            foreground='#16733c')
            else:
                frame = self.node.frame()
                camera = self.session.camera if self.session else None
                if self.undistort.get() and camera:
                    k,d,_,_ = camera
                    frame = cv2.undistort(frame, k, d)
                mounting = self.book.select() == str(self.mount_tab)
                columns, rows = (('floor_columns', 'floor_rows') if mounting else ('columns', 'rows'))
                frame, board_text, found = annotate_board(
                    frame, self.fields[columns].get(), self.fields[rows].get())
                board_text = ('장착용 · ' if mounting else '내부 보정용 · ')+board_text
                self.board_status.configure(text=board_text,
                                            foreground='#16733c' if found else '#a05a00')
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            image.thumbnail((550, 410))
            self.photo = ImageTk.PhotoImage(image)
            self.preview.configure(image=self.photo)
        except ValueError as error:
            if self.book.select() == str(self.ball_review):
                self.ball_review.no_frame(error)
            self.preview.configure(image='', text='JPEG 미리보기 미수신 · launch/ROS 설정 확인')
            self.board_status.configure(text='체커보드 인식: 카메라 수신 대기', foreground='#777777')
        self.root.after(150, self.refresh)

    def close(self):
        self.node.stop('UI 종료')
        if self.busy:
            messagebox.showinfo('실험 마무리', '주행을 정지했습니다. 계산/수집이 끝나면 창을 닫으세요.')
            return
        self.node.close()
        if self.session:
            self.session.save()
            self.session.release()
        self.root.destroy()


def main(args=None):
    from .ui_options import parse_ui_options
    options, ros_args = parse_ui_options(sys.argv[1:] if args is None else args)
    webcam = options.webcam is not None
    node = None
    if not webcam:
        rclpy.init(args=ros_args)
    try:
        if webcam:
            from .webcam_acquisition import WebcamAcquisition
            node = WebcamAcquisition(options.webcam, options.webcam_width, options.webcam_height,
                                     options.webcam_fps, options.webcam_hfov_deg)
        else:
            node = Acquisition()
        root = tk.Tk()
        App(root, node)
        root.mainloop()
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2
    finally:
        if node is not None and node.running:
            node.close()
        if not webcam:
            rclpy.try_shutdown()
