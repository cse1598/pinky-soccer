"""Read-only ball inspection tab driven by shared gameplay algorithms."""

from copy import deepcopy
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
import tkinter as tk
import uuid

from pinky_soccer_world_model.ball_detector import load_camera_yaml

import yaml

from .ball_review import compare_bumper_ball, GameplayReview, gameplay_settings
from .project_store import utc


class BallReviewView(ttk.Frame):
    """Inspect JPEG candidates, metric projection and gameplay BallFilter tracking."""

    def __init__(self, parent, on_saved=None, is_busy=None):
        super().__init__(parent, padding=10)
        self.on_saved = on_saved or (lambda: None)
        self.is_busy = is_busy or (lambda: False)
        self.engine = GameplayReview()
        self.external = None
        self.external_path = ''
        self.external_mount = None
        self.source_node = None
        self.report = None
        self.comparison = None
        self.session = None
        canvas = tk.Canvas(self, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(fill='both', expand=True)
        body = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=body, anchor='nw')
        body.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        self.intro = ttk.Label(body, text='경기 공 검출·거리 추정·BallFilter를 재사용하는 검토 모드입니다.\n'
                  '왼쪽 영상에 공 후보와 중심을 표시합니다. 기록 시작 없이 사용할 수 있습니다.',
                  wraplength=540)
        self.intro.pack(fill='x', pady=6)
        row = ttk.Frame(body)
        row.pack(fill='x')
        ttk.Button(row, text='카메라 보정 YAML/프로필 선택', command=self.load_camera).pack(side='left')
        ttk.Button(row, text='자동 보정 선택', command=self.auto_camera).pack(side='left')
        ttk.Button(body, text='경기 검출/월드모델 설정 YAML 불러오기', command=self.load_config).pack(fill='x', pady=5)
        self.config_text = ttk.Label(body, text='설정: 경기 패키지 ball_detector.yaml / world_model.yaml\n'
                                     f"공 지름 {self.engine.settings['ball_radius']*2000:.2f}mm (설정값)",
                                     wraplength=540)
        self.config_text.pack(fill='x')
        self.approximate = tk.BooleanVar(value=True)
        ttk.Checkbutton(body, text='보정값이 없으면 미보정 거리 추정 (가정값 사용)',
                        variable=self.approximate).pack(anchor='w')
        row = ttk.Frame(body)
        row.pack(fill='x')
        ttk.Label(row, text='미보정 가정 HFOV (°)').pack(side='left')
        self.hfov = tk.StringVar(value=str(self.engine.settings['approximate_hfov_deg']))
        ttk.Entry(row, textvariable=self.hfov, width=9).pack(side='left', padx=6)
        self.rotated = tk.BooleanVar(value=self.engine.settings['image_rotated_180'])
        self.rotated_toggle = ttk.Checkbutton(row, text='경기 설정: 영상 180° 반전', variable=self.rotated)
        self.rotated_toggle.pack(side='left')
        self.calibration = ttk.Label(body, text='카메라 보정: 영상 수신 대기', wraplength=540, padding=6)
        self.calibration.pack(fill='x')
        self.observation = ttk.Label(body, text='현재 공 관측: 대기', wraplength=540, padding=6)
        self.observation.pack(fill='x')
        self.tracking = ttk.Label(body, text='공 추적: 대기', wraplength=540, padding=6)
        self.tracking.pack(fill='x')
        self.geometry = ttk.Label(body, text='', wraplength=540, padding=6)
        self.geometry.pack(fill='x')
        box = ttk.LabelFrame(body, text='범퍼 기준 실측 비교 · 검토용 오프셋', padding=6)
        box.pack(fill='x', pady=6)
        ttk.Label(box, text='범퍼 앞면 바깥 중앙의 바닥 투영점 → 공 중심 (m). X 전방+, Y 왼쪽+.\n'
                  '공 표면까지 잰 거리는 중심까지로 환산하세요. 오프셋은 인식값에 더합니다.\n'
                  '경기 좌표/TF에는 적용하지 않습니다. 웹캠 모드에서는 사용할 수 없습니다.',
                  wraplength=520).pack(fill='x')
        self.actual_xy = [tk.StringVar(value=''), tk.StringVar(value='0')]
        self.offset_xy = [tk.StringVar(value='0'), tk.StringVar(value='0')]
        for title, variables in [('실측', self.actual_xy), ('오프셋', self.offset_xy)]:
            row = ttk.Frame(box)
            row.pack(fill='x')
            ttk.Label(row, text=title, width=9).pack(side='left')
            for axis, variable in zip(('X (m)', 'Y (m)'), variables):
                ttk.Label(row, text=axis).pack(side='left')
                ttk.Entry(row, textvariable=variable, width=10).pack(side='left', padx=4)
        row = ttk.Frame(box)
        row.pack(fill='x', pady=4)
        ttk.Button(row, text='현재 실측 차이를 오프셋으로', command=self.use_offset).pack(side='left')
        ttk.Button(row, text='비교 기록 저장', command=self.save_comparison).pack(side='left')
        self.comparison_text = ttk.Label(box, text='실측 X/Y를 입력하세요.', wraplength=520)
        self.comparison_text.pack(fill='x')
        self.description = ttk.Label(body, text='거리 = 표시 좌표 원점에서 공 중심까지 수평 거리. X 전방+, Y 왼쪽+.\n'
                  '미보정 값은 HFOV·왜곡 0·실행 TF 가정에 따른 추정입니다.\n'
                  '보정 높이/설치각도를 우선 사용하며 X/Y/yaw는 실행 TF를 유지합니다.\n'
                  '영상은 JPEG 미리보기이므로 경기 원본 영상과 픽셀 검출 차이가 날 수 있습니다.',
                  wraplength=540)
        self.description.pack(fill='x', pady=8)

    def configure_source(self, node):
        """Source is fixed by the startup flag; this does not add a GUI source switch."""
        self.source_node = node
        if getattr(node, 'source_kind', '') == 'webcam':
            self.engine.settings['floor_frame'] = node.floor_frame
            self.engine.settings['output_frame'] = node.floor_frame
            self.engine.stable_frames = (node.floor_frame,)
            self.approximate.set(node.hfov is not None)
            self.hfov.set('' if node.hfov is None else str(node.hfov))
            self.rotated.set(False)
            self.rotated_toggle.configure(state='disabled')
            self.config_text.configure(text='웹캠 리허설 · 경기 공통 검출/추적 알고리즘\n'
                                       '거리 원점: 웹캠 중심의 바닥 수직 투영점 / 단위 m')
            self.intro.configure(text='웹캠 리허설: 내부 보정·독립 검증 → 장착 탭의 바닥 높이/기울기 측정\n'
                                      '→ 공 인식 검토. 노트북/웹캠을 고정한 상태에서 확인하세요.')
            self.description.configure(text='거리 = 웹캠 바닥 투영점에서 공 중심까지 수평 거리.\n'
                                            '장착 탭에서 측정한 높이·roll/pitch를 로컬 좌표계에 사용합니다.\n'
                                            '내부값/높이/각도가 없으면 임의 생성하지 않습니다.\n'
                                            '노트북 화면 각도나 웹캠 위치를 바꾸면 장착 보정을 다시 하세요.')

    def auto_camera(self):
        """Return to project / CameraInfo / explicit approximation precedence."""
        self.external = None
        self.external_path = ''
        self.external_mount = None
        self.engine.reset()

    def load_camera(self):
        """Use the gameplay camera/profile loader without applying it to the robot."""
        path = filedialog.askopenfilename(title='카메라 내부 보정 또는 프로필',
                                         filetypes=[('YAML', '*.yaml *.yml')])
        if path:
            try:
                self.external = load_camera_yaml(path)
                data = yaml.safe_load(Path(path).read_text())
                self.external_mount = data.get(
                    'camera_mount', data.get('results', {}).get('camera_mount'))
                self.external_path = path
                self.engine.reset()
            except (OSError, ValueError, KeyError, TypeError) as error:
                messagebox.showerror('보정 파일 확인', str(error))

    def load_config(self):
        """Read the same ROS parameter file used for the game under inspection."""
        path = filedialog.askopenfilename(title='경기 검출/월드모델 설정',
                                         filetypes=[('YAML', '*.yaml *.yml')])
        if path:
            try:
                settings, params = gameplay_settings(path)
                external = load_camera_yaml(settings['camera_yaml']) if settings.get('camera_yaml') else None
                self.engine = GameplayReview(settings, params)
                self.rotated.set(settings['image_rotated_180'])
                self.hfov.set(str(settings['approximate_hfov_deg']))
                self.config_text.configure(text='검토 설정: '+path+
                                           f"\n공 지름 {settings['ball_radius']*2000:.2f}mm (설정값)")
                self.external = external
                self.external_path = settings.get('camera_yaml', '')
                self.external_mount = None
                if self.external_path:
                    data = yaml.safe_load(Path(self.external_path).read_text())
                    self.external_mount = data.get(
                        'camera_mount', data.get('results', {}).get('camera_mount'))
                if self.source_node:
                    self.configure_source(self.source_node)
            except (OSError, ValueError, KeyError, TypeError) as error:
                messagebox.showerror('경기 설정 확인', str(error))

    def update_frame(self, node, session):
        """Return annotated JPEG pixels and update diagnostics on the Tk thread."""
        if session is not self.session:
            previous = session.measurements.get('ball_review', []) if session else []
            latest = previous[-1] if previous else {}
            saved_offset = (session.meta.get('ball_review', {}).get('offset_xy_m')
                            if session else None)
            for variables, key, default in ((self.actual_xy, 'actual_xy_m', ['', '0']),
                                            (self.offset_xy, 'offset_xy_m', ['0', '0'])):
                values = (saved_offset if key == 'offset_xy_m' and saved_offset is not None
                          else latest.get(key, default))
                for variable, value in zip(variables, values):
                    variable.set(str(value))
        self.session = session
        self.comparison = None
        frame, header, info, key = node.review_frame()
        now = node.get_clock().now().nanoseconds * 1e-9
        observed = header.stamp.sec + header.stamp.nanosec * 1e-9
        if observed <= 0:
            observed = now
        preview, report = self.engine.process(
            frame, key, observed, now, header, node.review_transform, session, info, self.external,
            self.approximate.get(), self.hfov.get(), self.rotated.get(), self.external_mount)
        if getattr(node, 'source_kind', '') == 'webcam':
            report['mount_source'] = ('웹캠 바닥 보정 높이/roll/pitch · 노트북/웹캠을 움직이면 다시 측정하세요.\n'
                                      'webcam_floor: 원점은 웹캠 바닥 투영점, X는 카메라 전방, Y는 왼쪽')
            if report['reason'].startswith('카메라/바닥/'):
                report['reason'] = '웹캠 높이·기울기 측정이 필요합니다(장착 탭).'
        report['observed_at_s'] = observed
        report['frame_key'] = str(key)
        self.report = report
        self.calibration.configure(text='카메라: '+report['intrinsics_source']+
                                   ('\n'+self.external_path if self.external else ''),
                                   foreground=('#166040' if report['intrinsics_available']
                                               and not report['approximate'] else '#a05a00'))
        text = report['reason']
        if report['blob']:
            u, v, radius, area = report['blob']
            text += f'\n픽셀 중심 ({u:.1f}, {v:.1f}) · 반경 {radius:.1f}px · 색단서 면적 {area:.0f}px²'
            text += (f"\n영상 신뢰 점수 {report['visual_confidence']:.2f} · "
                     f"원호 지지 {report['visible_arc_fraction']*100:.0f}%"
                     + (' · 가림/반사광 윤곽 복원' if report['partial_outline'] else ''))
        if report['point']:
            x, y, z = report['point']
            text += (f"\n현재 관측 [{report['frame']}] X {x:.3f}m / Y {y:.3f}m / Z {z:.3f}m"
                     f"\n수평 거리 {report['range_m']:.3f}m · 방위각 {report['bearing_deg']:.1f}°")
        self.observation.configure(text=text)
        tracked = report['tracking']
        text = report['tracking_reason']
        if tracked:
            state = {'OBSERVED': '관측', 'ESTIMATED': '예측', 'LOST': '분실'}[tracked['state']]
            text += (f"\n{state}({tracked['state']}) · {tracked['stable_frame']}에서 추적 → {report['frame']} 표시"
                     f"\nX {tracked['point'][0]:.3f}m / Y {tracked['point'][1]:.3f}m · "
                     f"거리 {tracked['range_m']:.3f}m\n필터 모델 σ {tracked['sigma_m']:.3f}m · "
                     f"마지막 채택 관측 후 {tracked['age_s']:.2f}초")
        self.tracking.configure(text=text)
        geometry_text = f'영상 {frame.shape[1]}×{frame.shape[0]} · {header.frame_id}\n'+report['mount_source']
        comparison_text = '실측 X/Y를 입력하고 새 공 관측을 기다리세요.'
        if getattr(node, 'source_kind', '') == 'webcam':
            comparison_text = '웹캠 리허설에는 로봇 범퍼 기준이 없습니다.'
        elif report['point'] and report['frame'] == 'base_link' and abs(now-observed) <= 2:
            try:
                geometry = node.measurement_geometry()
                front = geometry.front
                x, y = report['point'][:2]
                geometry_text += f'\n범퍼 앞면 중앙 기준 공 중심: X {x-front[0]:.3f}m / Y {y-front[1]:.3f}m'
                comparison = compare_bumper_ball(
                    report['point'], front, [float(v.get()) for v in self.actual_xy],
                    [float(v.get()) for v in self.offset_xy])
                comparison.update(robot_description_sha256=geometry.sha256,
                                  geometry_source='running_robot_description_model_not_physical_measurement')
                self.comparison = comparison
                ex, ey = comparison['error_xy_m']
                cx, cy = comparison['corrected_xy_m']
                comparison_text = (
                    f'인식−실측: X {ex*1000:+.1f}mm / Y {ey*1000:+.1f}mm · 위치 오차 {comparison["error_m"]*1000:.1f}mm'
                    f'\n실측 수평 거리 {comparison["actual_range_m"]:.3f}m / 인식 {comparison["observed_range_m"]:.3f}m'
                    f'\n오프셋 적용: X {cx:.3f}m / Y {cy:.3f}m · 수평 거리 {comparison["corrected_range_m"]:.3f}m'
                    f'\n적용 후 위치 오차 {comparison["corrected_error_m"]*1000:.1f}mm')
            except (ValueError, TypeError) as error:
                comparison_text = '실측/범퍼 기준 확인: '+str(error)
        self.comparison_text.configure(text=comparison_text)
        self.geometry.configure(text=geometry_text)
        return preview

    def current_comparison(self):
        """Revalidate edited entries and stream freshness before a button action."""
        if self.comparison is None or self.report is None or self.source_node is None:
            raise ValueError('새 공 관측과 범퍼 기준·실측 입력이 필요합니다.')
        now = self.source_node.get_clock().now().nanoseconds * 1e-9
        if abs(now-self.report['observed_at_s']) > 2:
            raise ValueError('영상이 오래되었습니다. 새 관측을 기다리세요.')
        result = compare_bumper_ball(
            self.report['point'], self.comparison['bumper_front_base_xyz_m'],
            [float(v.get()) for v in self.actual_xy], [float(v.get()) for v in self.offset_xy])
        result.update(robot_description_sha256=self.comparison['robot_description_sha256'],
                      geometry_source=self.comparison['geometry_source'])
        return result

    def use_offset(self):
        """Copy the measured residual into the editable review-only correction."""
        try:
            result = self.current_comparison()
            for variable, value in zip(self.offset_xy, result['suggested_offset_xy_m']):
                variable.set(f'{value:.6f}')
        except (ValueError, TypeError) as error:
            messagebox.showerror('실측 비교', str(error))

    def save_comparison(self):
        """Persist validation evidence without invalidating calibration profiles."""
        try:
            if self.is_busy():
                raise ValueError('진행 중인 보정 작업이 끝난 뒤 저장하세요.')
            if self.session is None:
                raise ValueError('프로젝트를 먼저 열어 주세요.')
            result = self.current_comparison()
            result.update(id=uuid.uuid4().hex, recorded_utc=utc(),
                          observation=deepcopy(self.report),
                          camera_file=self.external_path, settings=deepcopy(self.engine.settings))
            self.session.measurements.setdefault('ball_review', []).append(result)
            self.session.meta.setdefault('ball_review', {})['offset_xy_m'] = result['offset_xy_m']
            self.session.revision += 1
            self.session.results.pop('ball_review', None)
            self.session.stale.discard('ball_review')
            self.session.save()
            self.on_saved()
            self.comparison_text.configure(text='비교 기록 저장 완료 · 측정 기록 탭에서 확인할 수 있습니다.')
        except (ValueError, TypeError, OSError) as error:
            messagebox.showerror('비교 기록 저장', str(error))

    def no_frame(self, error):
        """Clear stale observations instead of displaying the last frame as live."""
        self.engine.reset()
        self.report = None
        self.comparison = None
        self.comparison_text.configure(text='새 공 관측 수신 대기')
        self.calibration.configure(text='카메라 보정: 새 영상 수신 대기', foreground='#777777')
        self.observation.configure(text='공 관측 중단: '+str(error))
        self.tracking.configure(text='추적 초기화 · 새 영상 수신 대기')
        self.geometry.configure(text='')
