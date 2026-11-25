"""Offline motor analysis window (no robot needed): identification, plots, braking stats."""
import math
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import numpy as np

from . import motor_identification as mi

MEASURED, MODEL = '#1f77b4', '#d62728'


def braking_table(session):
    """Per commanded speed: odom distance / stop time statistics and manual distances."""
    groups = {}
    for row in session.measurements.get('braking', []):
        if row.get('source_deleted'):
            continue
        m = row.get('summary', {})
        if 'stop_time_s' not in m:
            continue
        v = round(abs(m['command_before_stop']['v_mps']), 3)
        g = groups.setdefault(v, {'odom': [], 'time': [], 'manual': [], 'from_step': 0})
        g['odom'].append(m['odom_braking_distance_m'])
        g['time'].append(m['stop_time_s'])
        if row.get('manual_distance_m') is not None:
            g['manual'].append(row['manual_distance_m'])
        g['from_step'] += bool(row.get('from_step_response'))
    lines = ['속도 m/s | n | odom 거리 평균±std / 최대 (cm) | 정지 시간 평균 / 최대 (s) | 실측 거리 (cm)']
    for v in sorted(groups):
        g = groups[v]
        odom, t = np.array(g['odom']) * 100, np.array(g['time'])
        sd = odom.std(ddof=1) if len(odom) > 1 else 0.
        manual = ', '.join(f'{x*100:.1f}' for x in g['manual']) or '-'
        lines.append(f'{v:.3f} | {len(odom)} (계단응답 {g["from_step"]}) | {odom.mean():.2f}±{sd:.2f} / '
                     f'{odom.max():.2f} | {t.mean():.3f} / {t.max():.3f} | {manual}')
    return '\n'.join(lines) if groups else '분석된 제동 기록이 없습니다 (모터 탭의 제동 분석 먼저).'


def summary_text(result):
    if not result:
        return '식별 결과 없음'
    lines = [f"모델: {result['model_version']}", f"시간축: {', '.join(result['timelines'])}",
             f"식별 {len(result['identification_experiments'])}건 · 독립 검증 "
             f"{len(result['validation_experiments'])}건 · "
             + ('검증됨' if result.get('validated') else '미검증 (독립 검증 자료 없음)'),
             '', '모델 | RMSE rad/s | 최대 | 파라미터 (상태) | 검증 RMSE 평균']
    for name, m in result.get('models', {}).items():
        if m.get('status') != 'fitted':
            lines.append(f'{name} | {m.get("status")}')
            continue
        params = ', '.join(f'{k}={v:.4g} ({m["parameter_status"].get(k, "")[:12]})'
                           for k, v in m['parameters'].items())
        val = m.get('validation') or []
        vr = f"{np.mean([v['rmse'] for v in val]):.3f}" if val else '-'
        lines.append(f"{name} | {m['rmse']:.3f} | {m['max_abs_error']:.3f} | {params} | {vr}")
    if result.get('best_on_validation'):
        lines.append(f"\n검증 데이터 기준 최선: {result['best_on_validation']}")
    for stage, s in (result.get('stages') or {}).items():
        lines.append(f"단계 {stage}: " + ', '.join(f'{k}={v:.4g}' for k, v in
                                                   (s.get('parameters') or {}).items())
                     + f" · RMSE {s.get('rmse', float('nan')):.3f}")
    lines.append('\nMPC에 자동 적용하지 않습니다 (identification.yaml은 별도 승인 단계용).')
    return '\n'.join(lines)


class MotorAnalysis(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app.root)
        self.app = app
        self.title('모터 응답 분석 (저장된 자료)')
        self.geometry('1100x780')
        top = ttk.Frame(self, padding=6)
        top.pack(fill='x')
        self.tree = ttk.Treeview(top, columns=('kind', 'cmd', 'purpose', 'status', 'timeline'),
                                 show='headings', height=8, selectmode='extended')
        for col, text, width in (('kind', '종류', 110), ('cmd', '명령 v / w', 140),
                                 ('purpose', '용도', 90), ('status', '상태', 110),
                                 ('timeline', '진단 자료', 160)):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width)
        self.tree.pack(side='left', fill='x', expand=True)
        buttons = ttk.Frame(top, padding=4)
        buttons.pack(side='left', fill='y')
        ttk.Button(buttons, text='선택(없으면 전체)으로 식별·검증',
                   command=self.identify).pack(fill='x', pady=2)
        self.model = tk.StringVar(value='ramp_first_order_delay')
        ttk.Combobox(buttons, textvariable=self.model, values=list(mi.MODELS),
                     state='readonly', width=24).pack(fill='x', pady=2)
        ttk.Button(buttons, text='선택 실험 그래프', command=self.plot).pack(fill='x', pady=2)
        ttk.Button(buttons, text='반복별 식별 비교', command=self.per_run).pack(fill='x', pady=2)
        ttk.Button(buttons, text='제동 통계', command=self.braking).pack(fill='x', pady=2)
        body = ttk.Panedwindow(self, orient='vertical')
        body.pack(fill='both', expand=True)
        self.figure_frame = ttk.Frame(body)
        self.text = ScrolledText(body, height=12, wrap='none', font=('monospace', 9))
        body.add(self.figure_frame, weight=3)
        body.add(self.text, weight=2)
        self.canvas = None
        self.refresh()

    def session(self):
        return self.app.require()

    def refresh(self):
        self.tree.delete(*self.tree.get_children())
        for row in self.session().measurements.get('motor_response', []):
            c = row.get('plan', {}).get('commanded', {})
            diag = 'motor/diagnostics' if any(row.get('samples', {}).get('/motor/diagnostics', 0)
                                              for _ in [0]) else 'joint_states만'
            self.tree.insert('', 'end', iid=row['id'], values=(
                row.get('plan', {}).get('kind'), f"{c.get('v_mps', 0):+.2f} / {c.get('w_radps', 0):+.2f}",
                '검증용' if row.get('purpose') == 'validation' else '식별용', row.get('status'), diag))
        self.show(summary_text(self.session().results.get('motor_identification')))

    def show(self, text):
        self.text.delete('1.0', 'end')
        self.text.insert('1.0', text)

    def selection(self):
        return set(self.tree.selection()) or None

    def identify(self):
        selection = self.selection()
        self.show('식별 계산 중 … (실험 수에 따라 수십 초)')
        self.app.job(lambda: (self.session().recalculate_derived('motor_identification', selection),
                              '모터 식별 완료')[1])
        self.after(500, self.wait_done)

    def wait_done(self):
        if self.app.busy:
            self.after(500, self.wait_done)
        else:
            self.refresh()

    def load(self, rid):
        session = self.session()
        row = next(r for r in session.measurements['motor_response'] if r['id'] == rid)
        return mi.load_run(session.local_path(row['files']['folder']), row)

    def plot(self):
        ids = list(self.tree.selection())
        if not ids:
            self.show('그래프로 볼 실험을 하나 이상 선택하세요.')
            return
        import matplotlib
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        # Korean labels: first installed CJK font wins, otherwise the default font.
        matplotlib.rcParams['font.family'] = ['Noto Sans CJK KR', 'NanumGothic',
                                              'Noto Serif CJK KR', 'DejaVu Sans']
        matplotlib.rcParams['axes.unicode_minus'] = False
        result = self.session().results.get('motor_identification') or {}
        model = self.model.get()
        params = (result.get('models', {}).get(model) or {}).get('parameters')
        run = self.load(ids[0])
        fig = Figure(figsize=(10, 4.6), dpi=90)
        axes = [fig.add_subplot(2, 2, k) for k in (1, 2, 3, 4)]
        for i, side in ((0, '좌'), (1, '우')):
            ax, rax = axes[i], axes[2 + i]
            if run.goal is not None:
                ax.step(run.goal.t, run.goal.wheel(i), where='post', color='#555555', label='명령 (Goal)')
            if run.trajectory is not None:
                ax.plot(run.trajectory.t, run.trajectory.wheel(i), '.', color='#2ca02c', ms=3,
                        label='내부 램프 (측정)')
            if run.present is not None:
                ax.plot(run.present.t, run.present.wheel(i), '.', color=MEASURED, ms=3, label='실제 속도 (측정)')
            if params and run.present is not None:
                ramp, delay = mi.MODELS[model]
                pred, ref = mi.simulate(run.goal.t, run.goal.wheel(i), run.present.t,
                                        params['tau_left' if i == 0 else 'tau_right'],
                                        params.get('motor_accel') if ramp else None,
                                        params.get('delay', 0.) if delay else 0.)
                ax.plot(run.present.t, pred, color=MODEL, label=f'모델 예측 ({model})')
                if ramp and run.trajectory is None:
                    ax.plot(run.present.t, ref, '--', color='#9467bd', label='램프 (모델 추정)')
                rax.plot(run.present.t, pred - run.present.wheel(i), color=MODEL)
                rax.axhline(0, color='#999999', lw=.5)
            ax.set_title(f'{side} 바퀴 · {run.timeline} · {run.kind}')
            ax.set_ylabel('rad/s')
            rax.set_ylabel('잔차 rad/s')
            rax.set_xlabel('s (실험 시작 기준, 기록된 시각)')
            ax.legend(fontsize=7)
        fig.tight_layout()
        if self.canvas:
            self.canvas.get_tk_widget().destroy()
        self.canvas = FigureCanvasTkAgg(fig, master=self.figure_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        q = run.quality
        self.show(summary_text(result) + '\n\n자료 품질 ' + run.experiment_id[:8] + ': '
                  + ', '.join(f'{k}={v}' for k, v in q.items()))

    def per_run(self):
        """Fit the selected model on each run separately (spread across repeats)."""
        ids = list(self.tree.selection()) or [r['id'] for r in self.session().measurements.get(
            'motor_response', []) if r.get('status') == 'complete']
        model = self.model.get()
        lines = [f'반복별 식별 ({model})', '실험 | ' + ' | '.join(['tau_L', 'tau_R', 'accel', 'delay', 'RMSE'])]
        values = []
        for rid in ids:
            try:
                fit = mi.fit(model, [self.load(rid)])
            except Exception as error:          # one bad run must not stop the comparison
                lines.append(f'{rid[:8]} | 오류: {error}')
                continue
            if fit.get('status') != 'fitted':
                lines.append(f"{rid[:8]} | {fit.get('status')}")
                continue
            p = fit['parameters']
            values.append(p)
            lines.append(f"{rid[:8]} | " + ' | '.join(
                f"{p[k]:.4g}" if k in p else '-' for k in ('tau_left', 'tau_right', 'motor_accel', 'delay'))
                + f" | {fit['rmse']:.3f}")
        if len(values) > 1:
            for k in values[0]:
                v = np.array([p[k] for p in values])
                lines.append(f'{k}: 평균 {v.mean():.4g} · std {v.std(ddof=1):.3g} · '
                             f'변동계수 {v.std(ddof=1) / max(abs(v.mean()), 1e-12):.2f}')
        self.show('\n'.join(lines))

    def braking(self):
        self.show(braking_table(self.session()))
