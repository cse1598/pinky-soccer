"""Read-only ball inspection tab driven by shared gameplay algorithms."""

from tkinter import filedialog, messagebox, ttk
import tkinter as tk

from pinky_soccer_world_model.ball_detector import load_camera_yaml

from .ball_review import GameplayReview, gameplay_settings


class BallReviewView(ttk.Frame):
    """Inspect JPEG candidates, metric projection and gameplay BallFilter tracking."""

    def __init__(self, parent):
        super().__init__(parent, padding=10)
        self.engine = GameplayReview()
        self.external = None
        self.external_path = ''
        self.source_node = None
        self.report = None
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
        self.description = ttk.Label(body, text='거리 = 표시 좌표 원점에서 공 중심까지 수평 거리. X 전방+, Y 왼쪽+.\n'
                  '미보정 값은 HFOV·왜곡 0·실행 TF 가정에 따른 추정입니다.\n'
                  '카메라 위치/각도는 실행 중 TF를 사용하며, 장착 측정값을 자동 적용하지 않습니다.\n'
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
        self.engine.reset()

    def load_camera(self):
        """Use the gameplay camera/profile loader without applying it to the robot."""
        path = filedialog.askopenfilename(title='카메라 내부 보정 또는 프로필',
                                         filetypes=[('YAML', '*.yaml *.yml')])
        if path:
            try:
                self.external = load_camera_yaml(path)
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
                if self.source_node:
                    self.configure_source(self.source_node)
            except (OSError, ValueError, KeyError, TypeError) as error:
                messagebox.showerror('경기 설정 확인', str(error))

    def update_frame(self, node, session):
        """Return annotated JPEG pixels and update diagnostics on the Tk thread."""
        frame, header, info, key = node.review_frame()
        now = node.get_clock().now().nanoseconds * 1e-9
        observed = header.stamp.sec + header.stamp.nanosec * 1e-9
        if observed <= 0:
            observed = now
        preview, report = self.engine.process(
            frame, key, observed, now, header, node.review_transform, session, info, self.external,
            self.approximate.get(), self.hfov.get(), self.rotated.get())
        if getattr(node, 'source_kind', '') == 'webcam':
            report['mount_source'] = ('웹캠 바닥 보정 높이/roll/pitch · 노트북/웹캠을 움직이면 다시 측정하세요.\n'
                                      'webcam_floor: 원점은 웹캠 바닥 투영점, X는 카메라 전방, Y는 왼쪽')
            if report['reason'].startswith('카메라/바닥/'):
                report['reason'] = '웹캠 높이·기울기 측정이 필요합니다(장착 탭).'
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
        if report['point'] and report['frame'] == 'base_link':
            try:
                front = node.measurement_geometry().front
                x, y = report['point'][:2]
                geometry_text += f'\n범퍼 앞면 중앙 기준 공 중심: X {x-front[0]:.3f}m / Y {y-front[1]:.3f}m'
            except ValueError:
                pass
        self.geometry.configure(text=geometry_text)
        return preview

    def no_frame(self, error):
        """Clear stale observations instead of displaying the last frame as live."""
        self.engine.reset()
        self.report = None
        self.calibration.configure(text='카메라 보정: 새 영상 수신 대기', foreground='#777777')
        self.observation.configure(text='공 관측 중단: '+str(error))
        self.tracking.configure(text='추적 초기화 · 새 영상 수신 대기')
        self.geometry.configure(text='')
