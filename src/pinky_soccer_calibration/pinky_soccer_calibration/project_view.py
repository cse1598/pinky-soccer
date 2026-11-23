"""Non-camera measurement history and an overview of computed project results."""

import json
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import yaml


class ProjectView(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent, padding=8)
        self.app = app
        self.heading = ttk.Label(self, text='새 프로젝트를 만들거나 저장 폴더를 불러오세요.', wraplength=600)
        self.heading.pack(fill='x')
        tabs = ttk.Notebook(self)
        tabs.pack(fill='both', expand=True)
        records, results = [ttk.Frame(tabs) for _ in range(2)]
        for frame, title in [(records, '실험 목록'), (results, '전체 계산 결과')]:
            tabs.add(frame, text=title)
        self.counts = ttk.Label(records, wraplength=600)
        self.counts.pack(fill='x')
        bar = ttk.Frame(records)
        bar.pack(fill='x')
        ttk.Button(bar, text='선택 기록 삭제', command=self.delete_records).pack(side='left')
        ttk.Button(bar, text='선택 항목 다시 계산', command=self.recalculate).pack(side='left')
        self.records = self.table(records, ('항목', '측정값', '시각'), (90, 330, 150), height=8)
        self.records.bind('<<TreeviewSelect>>', self.show_record)
        self.details = ScrolledText(records, height=10, wrap='word')
        self.details.pack(fill='both', expand=True)
        ttk.Label(results, text='데이터 변경 후 관련 결과는 재계산해야 합니다.', wraplength=580).pack(fill='x')
        self.results = ScrolledText(results, wrap='word', height=20)
        self.results.pack(fill='both', expand=True)
        self.record_rows = {}

    @staticmethod
    def table(parent, columns, widths, height):
        frame = ttk.Frame(parent)
        frame.pack(fill='both', expand=True)
        tree = ttk.Treeview(frame, columns=columns, show='headings', selectmode='extended', height=height)
        for name, width in zip(columns, widths):
            tree.heading(name, text=name)
            tree.column(name, width=width, minwidth=60)
        scroll = ttk.Scrollbar(frame, orient='vertical', command=tree.yview)
        tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        tree.pack(fill='both', expand=True)
        return tree

    def refresh(self):
        session = self.app.session
        if not session:
            return
        self.heading.configure(text=f"{session.meta['robot_id']} · {session.meta['environment']}\n{session.root}")
        self.records.delete(*self.records.get_children())
        self.record_rows = {}
        self.details.delete('1.0', 'end')
        groups = {'wheels': session.trials, **session.measurements}
        labels = {'wheels': '바퀴', 'floor': '장착', 'lidar': 'LiDAR', 'ultrasonic': '초음파',
                  'imu_stationary': 'IMU 정지', 'timing': '주기', 'ball_review': '공 실측 비교'}
        counts = {k: sum(v.get('kind') == k for v in session.trials)
                  for k in ('forward', 'backward', 'left', 'right')}
        self.counts.configure(text='바퀴: 전진 {forward} / 후진 {backward} / 좌회전 {left} / 우회전 {right}회'
                              .format(**counts)+'\n'+' · '.join(f'{labels.get(k,k)} {len(v)}회'
                                                             for k, v in groups.items() if k != 'wheels'))
        for category, rows in groups.items():
            for row in rows:
                if category == 'wheels':
                    detail = {'forward': '전진', 'backward': '후진', 'left': '좌회전', 'right': '우회전'}\
                        .get(row.get('kind'), '이전 실험')
                    if 'longitudinal_m' in row:
                        detail += f" X {row['longitudinal_m']:.4f}m / Y {row['lateral_m']*1000:.1f}mm"
                    elif row.get('measurement_type') == 'straight_distance':
                        detail += f" 시작~끝 거리 {row['distance_m']:.4f}m"
                    elif 'measured_angle_deg' in row:
                        detail += f" {row['measured_angle_deg']:.2f}°"
                    else:
                        detail += f" s={row.get('distance_m')}m / θ={row.get('angle_rad')}rad"
                elif category == 'floor':
                    detail = Path(row.get('image_file', '')).name
                elif category == 'ball_review':
                    detail = (f"위치 오차 {row['error_m']*1000:.1f}mm → "
                              f"오프셋 적용 후 {row['corrected_error_m']*1000:.1f}mm")
                else:
                    detail = ('이전 분석값(원시 표본 없음)' if 'legacy_report' in row else
                              f"표본 {len(row.get('samples', []))}개 · "+json.dumps(row.get('reference', {}),
                                                                                 ensure_ascii=False)[:70])
                iid = self.records.insert('', 'end', values=(labels.get(category, category), detail,
                                                           row.get('recorded_utc', '이전 기록')))
                self.record_rows[iid] = category, row
        self.results.configure(state='normal')
        self.results.delete('1.0', 'end')
        self.results.insert('end', '재계산 필요: '+(', '.join(sorted(session.stale)) or '없음')+'\n\n'+
                            yaml.safe_dump(session.results, allow_unicode=True, sort_keys=False))
        self.results.configure(state='disabled')

    def show_record(self, event=None):
        selection = self.records.selection()
        if selection:
            self.details.delete('1.0', 'end')
            self.details.insert('end', yaml.safe_dump(self.record_rows[selection[0]][1], allow_unicode=True,
                                                      sort_keys=False))

    def delete_records(self):
        selected = [self.record_rows[i] for i in self.records.selection()]
        if not selected or not messagebox.askyesno('기록 제외', '선택 기록을 계산에서 제외할까요? 원본 Bag/CSV는 보존합니다.'):
            return
        def remove():
            session = self.app.require()
            for category in {v[0] for v in selected}:
                session.remove_measurements(category, {v['id'] for k, v in selected if k == category})
        self.app.job(remove)

    def recalculate(self):
        selected = [self.record_rows[i][0] for i in self.records.selection()]
        if not selected:
            messagebox.showinfo('항목 선택', '다시 계산할 항목의 기록을 선택하세요.')
            return
        categories = set(selected)
        self.app.job(lambda: {key: self.app.require().recalculate(key) for key in sorted(categories)})
