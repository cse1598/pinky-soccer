"""Desktop camera calibration workbench over the robot's existing Jupyter server."""

import argparse
from pathlib import Path
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import cv2
import numpy as np
from PIL import Image, ImageTk

from .core import detect, load, positive
from .jupyter_camera import JupyterCamera
from .workbench_model import Experiment


def report_text(report):
    error = report.get('rms_px')
    prefix = '통과' if report['passed'] else '재촬영 필요'
    if not error:
        return prefix + '\n검출된 보드가 없습니다.'
    std = error['sample_std']
    text = (f'{prefix} · {error["count"]}장\n'
            f'평균 RMS {error["mean"]:.3f} px\n'
            f'표본 표준편차 {std:.3f} px\n최대 {error["max"]:.3f} px') if std is not None else prefix
    reasons = {
        'Insufficient position / distance / tilt diversity': '보드를 상하좌우·앞뒤로 옮기고 기울여 촬영하세요.',
        'One or more views exceed the configured RMS limit': '오차가 큰 사진이 있습니다. 흔들림·초점·코너를 확인하세요.',
        'Radial model folds within the image; recalibrate': '가장자리 왜곡 모델이 불안정합니다. 새 실험을 권장합니다.',
    }
    if report['reasons']:
        text += '\n\n' + '\n'.join(reasons.get(reason, reason) for reason in report['reasons'])
    return text


class Worker(threading.Thread):
    """Serialize all robot operations; Tk only receives messages on its own thread."""

    def __init__(self, events):
        super().__init__(daemon=True)
        self.events = events
        self.jobs = queue.Queue()
        self.remote = self.experiment = None
        self.stopped = threading.Event()
        self.auto = False
        self.purpose = 'calibration'
        self.last_shot = 0.

    def state(self):
        exp = self.experiment
        self.events.put(('state', {
            'connected': self.remote is not None, 'experiment': exp is not None,
            'counts': {key: len(value) for key, value in exp.samples.items()} if exp else {},
            'trained': bool(exp and (exp.train_report or {}).get('passed')),
            'validated': bool(exp and (exp.valid_report or {}).get('passed')),
            'exported': bool(exp and exp.exported), 'auto': self.auto,
            'purpose': self.purpose,
            'coverage': exp.coverage(self.purpose) if exp else '',
            'root': str(exp.root) if exp else None,
        }))

    def close(self):
        self.auto = False
        if self.experiment:
            self.experiment.close()
        if self.remote:
            self.remote.close()
            self.remote = None

    def run(self):
        try:
            while True:
                try:
                    action, params = self.jobs.get(timeout=.25 if self.remote else .5)
                except queue.Empty:
                    if self.remote:
                        try:
                            sample = self.remote.frame()
                            self.preview(sample)
                            if self.auto and time.monotonic() - self.last_shot >= 2:
                                self.last_shot = time.monotonic()
                                try:
                                    self.capture()
                                except ValueError as error:
                                    self.events.put(('status', str(error)))
                        except Exception as error:
                            self.close()
                            self.events.put(('error', '연결이 끊어졌습니다. 원본은 저장되어 있습니다.\n' + str(error)))
                            self.state()
                    continue
                try:
                    if action == 'stop':
                        return
                    if action == 'connect':
                        self.close()
                        self.experiment = None
                        self.remote = JupyterCamera(params['url'], params['token'])
                        info = self.remote.connect(params['width'], params['height'])
                        self.events.put(('status', f'{info["model"]} 연결 완료 · {params["width"]}×{params["height"]}'))
                    elif action == 'disconnect':
                        self.close()
                        self.events.put(('status', '카메라와 전용 커널을 종료했습니다.'))
                    elif action == 'new':
                        if self.remote is None:
                            raise ValueError('먼저 로봇에 연결하세요.')
                        self.auto = False
                        self.purpose = 'calibration'
                        if self.experiment:
                            self.experiment.close()
                        frame = self.remote.frame()['image']
                        self.experiment = Experiment(self.remote, size=frame.shape[:2][::-1], **params)
                        self.events.put(('status', '새 실험 시작 · 보드 전체를 보이게 하고 학습 사진을 촬영하세요.'))
                    elif action == 'purpose':
                        self.auto = False
                        self.purpose = params['purpose']
                    elif action == 'capture':
                        self.capture()
                    elif action == 'auto':
                        self.auto = not self.auto
                    elif action in ('fit', 'validate'):
                        self.auto = False
                        report = getattr(self.experiment, action)(max_rms=params['max_rms'])
                        if action == 'fit' and report['passed']:
                            self.purpose = 'validation'
                        self.events.put(('report', report_text(report)))
                        self.events.put(('status', ('학습' if action == 'fit' else '독립 검증') +
                                         (' 통과' if report['passed'] else ' 결과를 확인하고 보완 촬영하세요.')))
                    elif action == 'export':
                        output = self.experiment.export(params['visual_check'])
                        self.close()
                        self.events.put(('report', '검증된 결과를 저장했습니다.\n\n' + str(output)))
                        self.events.put(('status', '저장 완료 · 카메라 종료. 다음 단계는 결과 확인 후 진행합니다.'))
                    self.state()
                except Exception as error:
                    self.auto = False
                    if action == 'connect':
                        self.close()
                    self.events.put(('error', str(error)))
                    self.state()
                finally:
                    self.events.put(('busy', False))
        finally:
            self.close()
            self.stopped.set()

    def capture(self):
        if not self.experiment:
            raise ValueError('새 실험을 먼저 시작하세요.')
        sample = self.experiment.capture(self.purpose)
        self.preview(sample)
        self.events.put(('status', ('학습' if self.purpose == 'calibration' else '검증') +
                         ' 사진 저장 · 다음에는 위치·거리·기울기를 바꿔 주세요.'))
        self.state()

    def preview(self, sample):
        frame = sample['image'].copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = detect(gray)
        if corners is not None:
            cv2.drawChessboardCorners(frame, (9, 6), corners, True)
        exp = self.experiment
        if exp and exp.camera_path and exp.camera_path.exists():
            data = load(exp.camera_path)
            k = np.array(data['camera_matrix']['data']).reshape(3, 3)
            d = np.array(data['distortion_coefficients']['data'])
            corrected = cv2.undistort(sample['image'], k, d)
        else:
            corrected = None
        self.events.put(('preview', (frame, corrected, corners is not None,
                                     float(cv2.Laplacian(gray, cv2.CV_64F).var()))))


class Workbench:
    def __init__(self, root, url, output):
        self.root = root
        self.events = queue.Queue()
        self.worker = Worker(self.events)
        self.current = {}
        self.busy = self.closing = False
        self.last_frame = self.corrected_frame = None
        root.title('Pinky Pro · 카메라 캘리브레이션')
        root.geometry('1160x980')
        root.minsize(1000, 960)
        root.configure(bg='#f4f6fa')
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=('Sans', 10), background='#f4f6fa')
        style.configure('TButton', padding=(10, 4))
        style.configure('Title.TLabel', font=('Sans', 20, 'bold'), foreground='#24314b')
        style.configure('Accent.TButton', background='#b83e68', foreground='white')
        style.map('Accent.TButton', background=[('active', '#963257')])
        style.configure('Card.TLabelframe', padding=8)
        body = ttk.Frame(root, padding=20)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='Pinky Pro  /  카메라 내부 보정', style='Title.TLabel').pack(anchor='w')
        ttk.Label(body, text='연결 → 다양한 자세의 학습 촬영 → 내장 보정 → 새 사진으로 검증 → 결과 저장').pack(anchor='w', pady=(4, 14))
        connection = ttk.LabelFrame(body, text='로봇 연결', style='Card.TLabelframe')
        connection.pack(fill='x')
        self.url, self.token = tk.StringVar(value=url), tk.StringVar()
        self.width, self.height = tk.StringVar(value='640'), tk.StringVar(value='480')
        self.field(connection, 'Jupyter 주소', self.url, 0, 0, 31)
        self.field(connection, '토큰 (필요할 때만)', self.token, 0, 2, 18, show='•')
        self.field(connection, '너비', self.width, 1, 0, 8)
        self.field(connection, '높이', self.height, 1, 2, 8)
        self.buttons = {}
        self.button(connection, 'connect', '연결', self.connect).grid(row=0, column=4, padx=8)
        self.button(connection, 'disconnect', '연결 해제', lambda: self.submit('disconnect')).grid(row=1, column=4, padx=8)
        options = ttk.LabelFrame(body, text='실험 설정 · 9×6 내부 코너', style='Card.TLabelframe')
        options.pack(fill='x', pady=12)
        self.square, self.camera_id = tk.StringVar(value='30'), tk.StringVar(value='pinky_ov5647')
        self.output, self.notes = tk.StringVar(value=str(Path(output).resolve())), tk.StringVar()
        self.max_rms = tk.StringVar(value='0.7')
        self.field(options, '한 칸 실측 (mm)', self.square, 0, 0, 8)
        self.field(options, '카메라 식별명', self.camera_id, 0, 2, 16)
        self.field(options, '오차 한계 (px)', self.max_rms, 0, 4, 8)
        self.field(options, '저장 폴더', self.output, 1, 0, 42)
        ttk.Button(options, text='폴더 선택', command=self.choose_output).grid(row=1, column=2, padx=8)
        self.button(options, 'new', '새 실험', self.new, accent=True).grid(row=1, column=5, padx=8)
        self.field(options, '측정 메모', self.notes, 2, 0, 42)
        main = ttk.Frame(body)
        main.pack(fill='both', expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, minsize=320)
        main.rowconfigure(0, weight=1)
        visual = ttk.Frame(main)
        visual.grid(row=0, column=0, sticky='nsew')
        self.image = tk.Label(visual, text='로봇에 연결하면 영상이 표시됩니다.', bg='#182332', fg='white',
                              width=70, height=18, anchor='center')
        self.image.pack(fill='both', expand=True)
        self.show_corrected = tk.BooleanVar()
        ttk.Checkbutton(visual, text='보정 후 영상 보기 (추정 완료 후)', variable=self.show_corrected,
                        command=self.render).pack(anchor='w', pady=(8, 2))
        self.detection = tk.StringVar(value='체커보드: 대기 중')
        ttk.Label(visual, textvariable=self.detection).pack(anchor='w')
        ttk.Label(visual, text='보드를 상하좌우·앞뒤로 옮기고 좌우·상하로 기울이세요.\n촬영할 때는 모든 코너가 보이도록 잠깐 멈춰 주세요.').pack(anchor='w', pady=6)
        side = ttk.Frame(main, padding=(18, 0, 0, 0))
        side.grid(row=0, column=1, sticky='nsew')
        self.stage = tk.StringVar(value='calibration')
        for label, value in [('1. 학습 촬영', 'calibration'), ('2. 독립 검증 촬영', 'validation')]:
            ttk.Radiobutton(side, text=label, variable=self.stage, value=value,
                            command=self.change_stage).pack(anchor='w', pady=3)
        self.counts = tk.StringVar(value='학습 0 / 권장 25장\n검증 0 / 최소 8장')
        ttk.Label(side, textvariable=self.counts, font=('Sans', 12, 'bold')).pack(anchor='w', pady=8)
        self.coverage = tk.StringVar()
        ttk.Label(side, textvariable=self.coverage, font=('Sans', 10)).pack(anchor='w', pady=(0, 6))
        self.button(side, 'capture', '사진 저장  [Space]', lambda: self.submit('capture'), accent=True).pack(fill='x', pady=3)
        self.button(side, 'auto', '2초 간격 자동 촬영', lambda: self.submit('auto')).pack(fill='x', pady=3)
        self.button(side, 'fit', '학습 보정 실행 (최소 15장)', lambda: self.analyze('fit')).pack(fill='x', pady=(12, 3))
        self.button(side, 'validate', '독립 검증 실행', lambda: self.analyze('validate')).pack(fill='x', pady=3)
        self.visual_check = tk.BooleanVar()
        self.visual_check_widget = ttk.Checkbutton(
            side, text='코너·왜곡 제거 영상 확인 완료', variable=self.visual_check)
        self.visual_check_widget.pack(anchor='w', pady=10)
        self.button(side, 'export', '검증된 결과 저장', self.export, accent=True).pack(fill='x', pady=3)
        self.button(side, 'folder', '사진·보고서 폴더 열기', self.open_folder).pack(fill='x', pady=3)
        self.result = tk.StringVar(value='보정 결과는 아직 없습니다.\n\n실측한 칸 크기를 확인하고\n새 실험을 시작하세요.')
        self.result_box = ScrolledText(visual, height=5, wrap='word', font=('Sans', 10),
                                       background='#f4f6fa', relief='flat')
        self.result_box.pack(fill='x', pady=6)
        self.result.trace_add('write', self.render_report)
        self.render_report()
        self.status = tk.StringVar(value='로봇 Wi-Fi 연결을 확인하고 연결 버튼을 누르세요.')
        ttk.Label(body, textvariable=self.status, wraplength=1080, foreground='#7f3151').pack(
            side='bottom', anchor='w', pady=(12, 0), before=main)
        root.bind('<space>', self.space)
        root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.worker.start()
        self.refresh()
        root.after(80, self.poll)

    @staticmethod
    def field(parent, label, variable, row, column, width, **kwargs):
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky='w', padx=(0, 8), pady=3)
        ttk.Entry(parent, textvariable=variable, width=width, **kwargs).grid(
            row=row, column=column+1, sticky='w', padx=(0, 12), pady=3)

    def button(self, parent, key, text, command, accent=False):
        widget = ttk.Button(parent, text=text, command=command,
                            style='Accent.TButton' if accent else 'TButton')
        self.buttons[key] = widget
        return widget

    def choose_output(self):
        path = filedialog.askdirectory(parent=self.root)
        if path:
            self.output.set(path)

    def connect(self):
        try:
            width, height = int(self.width.get()), int(self.height.get())
            if not (160 <= width <= 1920 and 120 <= height <= 1080):
                raise ValueError('지원 모드의 너비·높이를 입력하세요. 확인된 모드는 640×480입니다.')
            self.submit('connect', url=self.url.get(), token=self.token.get(), width=width, height=height)
        except ValueError as error:
            messagebox.showerror('설정 확인', str(error), parent=self.root)

    def new(self):
        try:
            square = positive(self.square.get())
        except ValueError:
            messagebox.showerror('설정 확인', '실측한 칸 크기를 양수로 입력하세요.', parent=self.root)
            return
        self.stage.set('calibration')
        self.visual_check.set(False)
        self.result.set('학습 사진을 촬영하세요.\n권장 25장 이상 · 최소 15장')
        self.submit('new', output=self.output.get(), square_mm=square,
                    camera_id=self.camera_id.get(), notes=self.notes.get())

    def change_stage(self):
        if self.busy or self.closing:
            self.stage.set(self.current.get('purpose', 'calibration'))
            return
        if self.stage.get() == 'validation' and not self.current.get('trained'):
            self.stage.set('calibration')
            self.status.set('학습 보정을 통과한 후 독립 검증을 진행하세요.')
            return
        self.submit('purpose', purpose=self.stage.get())

    def analyze(self, action):
        try:
            limit = positive(self.max_rms.get())
        except ValueError:
            messagebox.showerror('설정 확인', '오차 한계를 양수로 입력하세요.', parent=self.root)
            return
        self.visual_check.set(False)
        self.submit(action, max_rms=limit)

    def export(self):
        self.submit('export', visual_check=self.visual_check.get())

    def submit(self, action, **params):
        if self.busy or self.closing:
            return
        self.busy = True
        self.status.set({'fit': '로봇 내장 보정 실행 중입니다. 사진 수에 따라 1분 정도 걸릴 수 있습니다.',
                         'validate': '별도 촬영한 사진으로 검증 중입니다.',
                         'connect': '로봇 Jupyter와 카메라에 연결 중입니다.'}.get(action, '처리 중…'))
        self.worker.jobs.put((action, params))
        self.refresh()

    def refresh(self):
        state = self.current
        connected, active = state.get('connected'), state.get('experiment')
        count = state.get('counts', {})
        self.counts.set(f'학습 {count.get("calibration", 0)} / 권장 25장\n검증 {count.get("validation", 0)} / 최소 8장')
        self.coverage.set(state.get('coverage', ''))
        trained, validated, exported = state.get('trained'), state.get('validated'), state.get('exported')
        capturing = connected and active and not exported and (self.stage.get() == 'validation' or not trained)
        enabled = {
            'connect': not connected, 'disconnect': connected, 'new': connected,
            'capture': capturing, 'auto': capturing,
            'fit': connected and active and not trained and count.get('calibration', 0) >= 15,
            'validate': active and trained and count.get('validation', 0) >= 8 and not exported,
            'export': validated and not exported, 'folder': active,
        }
        for key, widget in self.buttons.items():
            widget.configure(state='normal' if enabled.get(key) and not self.busy and not self.closing else 'disabled')
        self.visual_check_widget.configure(
            state='normal' if validated and not exported and not self.busy and not self.closing else 'disabled')
        self.buttons['auto'].configure(text='자동 촬영 중지' if state.get('auto') else '2초 간격 자동 촬영')

    def space(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry, tk.Text)):
            return
        if str(self.buttons['capture']['state']) != 'disabled':
            self.submit('capture')
        return 'break'

    def render(self):
        frame = self.corrected_frame if self.show_corrected.get() and self.corrected_frame is not None else self.last_frame
        if frame is None:
            return
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        image.thumbnail((660, 330))
        self.photo = ImageTk.PhotoImage(image)
        self.image.configure(image=self.photo, text='', width=0, height=0)

    def render_report(self, *args):
        self.result_box.configure(state='normal')
        self.result_box.delete('1.0', 'end')
        self.result_box.insert('1.0', self.result.get())
        self.result_box.configure(state='disabled')

    def poll(self):
        while True:
            try:
                kind, data = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'preview':
                self.last_frame, self.corrected_frame, found, blur = data
                self.detection.set(f'9×6 코너: {"검출됨" if found else "보드 전체를 보여 주세요"} · 선명도 {blur:.0f}')
                self.render()
            elif kind == 'state':
                self.current = data
                self.stage.set(data.get('purpose', 'calibration'))
                self.refresh()
            elif kind == 'status':
                self.status.set(data)
            elif kind == 'report':
                self.result.set(data)
            elif kind == 'busy':
                self.busy = data
                self.refresh()
            elif kind == 'error':
                self.status.set(data)
                if not self.closing:
                    messagebox.showerror('확인 필요', data, parent=self.root)
        if self.closing and self.worker.stopped.is_set():
            self.root.destroy()
        else:
            self.root.after(80, self.poll)

    def open_folder(self):
        path = Path(self.current['root'])
        subprocess.Popen(['xdg-open', str(path)])

    def shutdown(self):
        self.closing = True
        self.status.set('데이터를 마무리하고 카메라와 전용 커널을 종료하는 중입니다…')
        self.worker.jobs.put(('stop', {}))
        self.refresh()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://192.168.4.1:8888')
    parser.add_argument('--output', default='calibration_data')
    args = parser.parse_args(argv)
    root = tk.Tk()
    Workbench(root, args.url, args.output)
    root.mainloop()


if __name__ == '__main__':
    main()
