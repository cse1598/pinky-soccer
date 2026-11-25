"""Motor-dynamics tab: run step-response experiments; records live in '측정 기록'."""
import tkinter as tk
from tkinter import ttk

from .motor_experiment import APPROVED_LIMITS, KINDS, SAFE_LIMITS


class MotorView(ttk.Frame):
    """Inputs only; the experiment itself runs in App.run_motor on a worker thread."""

    def __init__(self, master, app):
        super().__init__(master, padding=10)
        self.app = app
        ttk.Label(self, wraplength=520, justify='left', text=(
            '모터 동역학 계단응답 · 정지 → 일정 명령 → 정지. 로봇을 바닥에 놓고 주변을 비우세요.\n'
            '원본 Bag/CSV와 실험별 명령·센서 기록을 보존합니다. 식별·그래프는 저장된 자료로 나중에 '
            '계산합니다. MPC 설정은 바꾸지 않습니다. 정지는 상단 ■ 버튼.')).pack(fill='x')
        self.kind = tk.StringVar(value='forward')
        line = ttk.Frame(self)
        line.pack(fill='x', pady=4)
        ttk.Label(line, text='실험 종류', width=22).pack(side='left')
        box = ttk.Combobox(line, state='readonly', width=24,
                           values=[f'{k} · {v[0]}' for k, v in KINDS.items()])
        box.current(0)
        box.bind('<<ComboboxSelected>>', lambda _: self.kind.set(box.get().split(' ')[0]))
        box.pack(side='left')
        self.vars = {}
        for key, label, default in (
                ('speed', '선속도 크기 (m/s)', '0.05'), ('rate', '각속도 크기 (rad/s)', '0.5'),
                ('pre', '정지 준비 (s)', '1.0'), ('hold', '명령 유지 (s)', '2.0'),
                ('post', '정지 후 기록 (s)', '2.0'), ('repeats', '반복 횟수', '3'),
                ('max_distance', '최대 이동 거리 (m)', '0.5'), ('floor', '바닥 재질', '')):
            row = ttk.Frame(self)
            row.pack(fill='x', pady=1)
            ttk.Label(row, text=label, width=22).pack(side='left')
            self.vars[key] = tk.StringVar(value=default)
            ttk.Entry(row, textvariable=self.vars[key], width=14).pack(side='left')
        self.purpose = tk.StringVar(value='identification')
        row = ttk.Frame(self)
        row.pack(fill='x', pady=2)
        ttk.Label(row, text='자료 용도', width=22).pack(side='left')
        for value, text in (('identification', '식별용'), ('validation', '독립 검증용')):
            ttk.Radiobutton(row, text=text, value=value, variable=self.purpose).pack(side='left')
        ttk.Label(self, foreground='#777777', wraplength=520, text=(
            f"기본 제한 |v|≤{SAFE_LIMITS['v']} m/s, |w|≤{SAFE_LIMITS['w']} rad/s · 고속 승인 시 "
            f"{APPROVED_LIMITS['v']} / {APPROVED_LIMITS['w']} · 바퀴 0.9×103 rpm. "
            '권장: 직진 0.05/0.10/0.15/0.20, 후진 0.05/0.10, 회전 0.5/1.0/2.0, 복합 0.10+1.0')
                  ).pack(fill='x', pady=4)
        self.high_speed = tk.BooleanVar(value=False)
        ttk.Checkbutton(self, text='저속 실험 정상 완료 확인 · 고속 실험 승인',
                        variable=self.high_speed).pack(anchor='w')
        self.armed = tk.BooleanVar(value=False)
        self.armed_text = tk.StringVar(value='주변을 비우고 바닥에 배치했습니다. 주행 허용')
        ttk.Checkbutton(self, textvariable=self.armed_text, variable=self.armed).pack(anchor='w')
        ttk.Button(self, text='계단응답 실험 실행 (반복)',
                   command=lambda: app.job(app.run_motor)).pack(fill='x', pady=6)
        self.status = ttk.Label(self, text='프로젝트를 열고 기록 시작/재개 후 실행하세요.',
                                wraplength=520, justify='left')
        self.status.pack(fill='x')

    def inputs(self, namespace, robot_id):
        """Snapshot of the form (Tk thread)."""
        self.armed_text.set(f'로봇 {robot_id} · namespace {namespace or "(없음)"} · 주변을 비우고 '
                            '바닥에 배치했습니다. 주행 허용')
        out = {k: v.get() for k, v in self.vars.items()}
        out.update(kind=self.kind.get(), purpose=self.purpose.get(),
                   high_speed=self.high_speed.get(), armed=self.armed.get())
        return out
