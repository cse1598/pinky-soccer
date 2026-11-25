"""Connection / clock tab: user-entered IPs, saved separately from calibration results."""
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from . import connection_settings as cs

OK, BAD, UNKNOWN = '#16733c', '#b03a2e', '#777777'


class ConnectionView(ttk.Frame):
    """Inputs are read on the Tk thread; nothing here talks to ROS directly."""

    def __init__(self, master, app):
        super().__init__(master, padding=10)
        self.app = app
        self.settings = cs.load()
        self.vars = {}
        top = ttk.LabelFrame(self, text='PC 공통 설정 (시간 기준 서버)', padding=6)
        top.pack(fill='x')
        for key, label in (('pc_ip', 'PC IPv4'), ('pc_interface', 'PC 인터페이스'),
                           ('ros_domain_id', 'ROS_DOMAIN_ID'),
                           ('rmw_implementation', 'RMW_IMPLEMENTATION')):
            self.entry(top, key, label, getattr(self.settings, key))
        found = ', '.join(f'{n}={a}' for n, a in cs.local_interfaces()) or '(확인 불가)'
        ttk.Label(top, text='참고: 이 PC 인터페이스 ' + found + ' (자동으로 입력값을 바꾸지 않음)',
                  foreground=UNKNOWN, wraplength=520).pack(fill='x')
        rob = ttk.LabelFrame(self, text='측정 대상 로봇 (프로젝트 1개 = 로봇 1대)', padding=6)
        rob.pack(fill='x', pady=6)
        r = self.settings.robot
        for key, label in (('robot_id', '로봇 식별명'), ('ip', 'Pinky IPv4'),
                           ('namespace', 'ROS namespace (없으면 빈칸)'),
                           ('ssh_user', 'SSH 사용자'), ('interface', '로봇 인터페이스')):
            self.entry(rob, 'robot_'+key, label, getattr(r, key))
        row = ttk.Frame(self)
        row.pack(fill='x')
        ttk.Button(row, text='연결 설정 저장', command=self.save).pack(side='left')
        ttk.Button(row, text='설정 명령 복사', command=self.copy).pack(side='left', padx=4)
        self.problems = ttk.Label(self, text='', wraplength=540, justify='left')
        self.problems.pack(fill='x', pady=4)
        state = ttk.LabelFrame(self, text='상태 (각각 독립 판정)', padding=6)
        state.pack(fill='x')
        self.state_labels = {}
        for key, label in (('ros', 'ROS 2 연결 (로봇 토픽 수신)'), ('sensors', '필수 센서 수신'),
                           ('pc_abs', 'PC 절대 시간 동기화'), ('robot_abs', 'Pinky 절대 시간 동기화'),
                           ('robot_pc', 'Pinky가 지정 PC를 시간 기준으로 사용'),
                           ('relative', 'PC–Pinky 상대 동기화 품질'),
                           ('one_way', '단방향 지연 측정 가능 여부')):
            line = ttk.Frame(state)
            line.pack(fill='x')
            ttk.Label(line, text=label, width=34).pack(side='left')
            self.state_labels[key] = ttk.Label(line, text='-', foreground=UNKNOWN, wraplength=300)
            self.state_labels[key].pack(side='left', fill='x')
        self.detail = ttk.Label(self, text='', wraplength=540, justify='left',
                                foreground=UNKNOWN)
        self.detail.pack(fill='x', pady=4)
        rec = ttk.Frame(self)
        rec.pack(fill='x')
        self.sync_seconds = tk.StringVar(value='30')
        ttk.Label(rec, text='시간 동기화 측정 기록 (초)').pack(side='left')
        ttk.Entry(rec, textvariable=self.sync_seconds, width=6).pack(side='left', padx=4)
        ttk.Button(rec, text='측정 → 프로젝트에 기록 추가',
                   command=lambda: app.job(app.record_clock_sync)).pack(side='left')
        ttk.Label(self, text='설정·확인 명령 (사용자가 직접 실행, 관리자 권한 명령 포함)').pack(anchor='w')
        self.text = ScrolledText(self, height=12, wrap='none', font=('monospace', 9))
        self.text.pack(fill='both', expand=True)
        self.update_commands()

    def entry(self, parent, key, label, value):
        line = ttk.Frame(parent)
        line.pack(fill='x', pady=1)
        ttk.Label(line, text=label, width=26).pack(side='left')
        self.vars[key] = tk.StringVar(value=value)
        ttk.Entry(line, textvariable=self.vars[key], width=24).pack(side='left')

    def current(self):
        """Settings from the entry fields (Tk thread)."""
        s = self.settings
        for key in ('pc_ip', 'pc_interface', 'ros_domain_id', 'rmw_implementation'):
            setattr(s, key, self.vars[key].get().strip())
        for key in ('robot_id', 'ip', 'namespace', 'ssh_user', 'interface'):
            setattr(s.robot, key, self.vars['robot_'+key].get().strip())
        return s

    def save(self):
        s = self.current()
        problems = s.validate()
        path = cs.save(s)
        self.app.node.pc_ip = s.pc_ip
        self.update_commands()
        self.problems.configure(
            text=('저장: ' + str(path) + ('\n확인 필요: ' + '; '.join(problems) if problems else '')),
            foreground=BAD if problems else OK)

    def copy(self):
        self.update_commands()
        self.clipboard_clear()
        self.clipboard_append(self.text.get('1.0', 'end'))

    def update_commands(self):
        self.text.delete('1.0', 'end')
        self.text.insert('1.0', cs.commands(self.current()))

    def show(self, sync_state, fresh, required, sync_enabled):
        """Refresh the independent state rows (called from the Tk refresh loop)."""
        s = self.current()
        mism = cs.runtime_mismatches(s, getattr(self.app.node, 'namespace', ''))
        problems = s.validate()
        if mism or problems:
            self.problems.configure(text='; '.join(problems + mism) +
                                    ('\n→ namespace/DDS 변경은 UI를 다시 시작해야 적용됩니다 '
                                     '(아래 6번 명령).' if mism else ''), foreground=BAD)

        def put(key, ok, text):
            self.state_labels[key].configure(
                text=text, foreground=UNKNOWN if ok is None else (OK if ok else BAD))
        put('ros', fresh['any'], '수신 중' if fresh['any'] else '로봇 토픽 없음 (domain/namespace/DDS 확인)')
        missing = [t for t in required if not fresh.get(t)]
        put('sensors', not missing, '모두 수신' if not missing else '미수신: ' + ', '.join(missing))
        if not sync_enabled:
            for key in ('pc_abs', 'robot_abs', 'robot_pc', 'relative', 'one_way'):
                put(key, None, '리허설 모드: 장치 간 시계 비교 불필요')
            self.detail.configure(text='')
            return
        c = sync_state.get('check') or {}
        pc = sync_state.get('pc') or {}
        robot = sync_state.get('robot') or {}
        put('pc_abs', c.get('pc_absolute'),
            f"기준 {pc.get('reference', '?')} · stratum {pc.get('stratum', '?')} · "
            f"leap {pc.get('leap', '?')}" if pc.get('reference') else (pc.get('error') or '확인 중'))
        put('robot_abs', c.get('robot_absolute') if robot else None,
            ('수신 없음' if not robot else
             f"stratum {robot.get('stratum', '?')} · leap {robot.get('leap', '?')} · "
             f"상태 나이 {c.get('robot_status_age_s') or 0:.1f}s"))
        put('robot_pc', c.get('robot_uses_pc') if robot else None,
            f"선택 서버 {c.get('robot_selected_server')} / 지정 PC {s.pc_ip or '미입력'}")
        bound = c.get('relative_bound_s')
        put('relative', c.get('relative_verified'),
            (f'상한 {bound*1e3:.2f} ms (기준 {c.get("limit_s", .005)*1e3:.0f} ms) · 표본 '
             f'{c.get("samples")}' if bound is not None else c.get('reason', '정보 부족')))
        put('one_way', bool(c.get('relative_verified')),
            '가능 (상한 이내 오차 포함)' if c.get('relative_verified') else
            '불가: 왕복 지연만 측정 (같은 장치 시계)')
        terms = c.get('relative_terms') or {}
        self.detail.configure(text=(
            '상한 구성: ' + ', '.join(f'{k} {v*1e3:.3f} ms' for k, v in terms.items())
            if terms else c.get('reason', '')) + (
            f"\n로봇 상태: {'구버전 bringup(tracking만)' if robot.get('legacy_status') else 'sources/sourcestats 포함'}"
            if robot else '') + f"\n갱신 {time.strftime('%H:%M:%S')}")
