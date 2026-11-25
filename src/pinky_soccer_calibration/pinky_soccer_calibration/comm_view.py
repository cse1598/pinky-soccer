"""Communication-latency tab: ROS 2 ping-pong and drive-command-path analysis."""
import tkinter as tk
from tkinter import ttk


class CommView(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, padding=10)
        ttk.Label(self, wraplength=520, justify='left', text=(
            '① ROS 2 Ping-Pong: 진단 전용 메시지(실제 주행 명령과 별개). 왕복은 PC 시계만, 단방향은 '
            '시간 동기화가 실험 내내 확인된 경우에만 계산합니다. 로봇에 latency_echo(이 bringup)가 '
            '필요합니다.')).pack(fill='x')
        self.vars = {}
        for key, label, default in (('rate', '송신 주기 (Hz: 10/20/50)', '20'),
                                    ('duration', '측정 시간 (s)', '20'),
                                    ('payload', '추가 페이로드 (bytes)', '0'),
                                    ('repeats', '반복 횟수', '1')):
            row = ttk.Frame(self)
            row.pack(fill='x', pady=1)
            ttk.Label(row, text=label, width=26).pack(side='left')
            self.vars[key] = tk.StringVar(value=default)
            ttk.Entry(row, textvariable=self.vars[key], width=10).pack(side='left')
        ttk.Button(self, text='Ping-Pong 측정 → 기록 추가',
                   command=lambda: app.job(app.run_pingpong)).pack(fill='x', pady=4)
        ttk.Separator(self).pack(fill='x', pady=8)
        ttk.Label(self, wraplength=520, justify='left', text=(
            '② 실제 명령 경로: 모터 동역학 실험의 각 명령에 순번 태그(Twist.linear.z, 드라이버가 '
            '무시)를 넣고, 로봇 motor_diagnostics:=true 의 드라이버 수신·시리얼 쓰기 시각과 정확히 '
            '대응시킵니다. 시간상 가까운 메시지로 추정하지 않습니다. 모터 내부 적용 시점은 직접 '
            '관측하지 못하므로 보고하지 않습니다.')).pack(fill='x')
        ttk.Button(self, text='모터 실험 명령 경로 분석 → 기록 추가',
                   command=lambda: app.job(app.analyze_command_path)).pack(fill='x', pady=4)

    def inputs(self):
        return {k: v.get() for k, v in self.vars.items()}
