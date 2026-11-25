"""Collision-outline tab: measured bumper extremes vs URDF boxes vs the MPC disc cover.

Frame: x forward, y left, origin = wheel-axle centre (MPC pose origin, base_footprint).
Extremes alone do not prove the whole outline; extra outline points can be added. Nothing
is applied to the MPC: results are stored as 'footprint' records only.
"""
import math
import tkinter as tk
from tkinter import ttk

try:                                     # same constants as the avoidance NMPC (read-only)
    from pinky_soccer_mpc.collision_geometry import FOOTPRINT_DISCS, FOOTPRINT_RECTS
except ImportError:                      # pragma: no cover - MPC package not built
    FOOTPRINT_DISCS, FOOTPRINT_RECTS = (), ()


def parse_points(text):
    """'x,y; x,y' (m) -> [(x, y)]."""
    out = []
    for item in text.replace('\n', ';').split(';'):
        item = item.strip()
        if item:
            x, y = (float(v) for v in item.split(','))
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('외곽점 값이 올바르지 않습니다.')
            out.append((x, y))
    return out


def evaluate(front, rear, left, right, origin_offset_x=0., points=()):
    """Measured outline points in the MPC frame and their disc-cover margins (m)."""
    for name, v in (('전방', front), ('후방', rear), ('좌측', left), ('우측', right)):
        if not (math.isfinite(v) and 0 < v < .5):
            raise ValueError(f'{name} 거리는 0~0.5 m 입니다.')
    # measured relative to the axle centre; shift into the MPC origin if they differ
    corners = [(front, left), (front, -right), (-rear, -right), (-rear, left)]
    pts = [(x - origin_offset_x, y) for x, y in corners + list(points)]

    def margin(x, y):
        return max(r - math.hypot(x - cx, y - cy) for cx, cy, r in FOOTPRINT_DISCS)
    # Bounding-box corners are NOT necessarily robot surface points (the Pinky outline is a
    # cross: wheels stick out only near the axle), so they are a conservative reference;
    # coverage is judged on the measured outline points the user lists.
    bbox = [margin(x, y) for x, y in pts[:4]] if FOOTPRINT_DISCS else []
    margins = [margin(x, y) for x, y in pts[4:]] if FOOTPRINT_DISCS else []
    urdf = None
    if FOOTPRINT_RECTS:
        urdf = {'front_m': max(r[2] for r in FOOTPRINT_RECTS), 'rear_m': -min(r[0] for r in FOOTPRINT_RECTS),
                'left_m': max(r[3] for r in FOOTPRINT_RECTS), 'right_m': -min(r[1] for r in FOOTPRINT_RECTS)}
    return {'points_mpc_frame': pts, 'measured': {'front_m': front, 'rear_m': rear,
                                                   'left_m': left, 'right_m': right},
            'urdf_extremes': urdf,
            'bbox_corner_min_margin_m': min(bbox) if bbox else None,
            'bbox_covered_by_discs': (min(bbox) >= 0) if bbox else None,
            'disc_cover_min_margin_m': min(margins) if margins else None,
            'discs_cover_measured_points': (min(margins) >= 0) if margins else None,
            'outline_complete': False if not points else 'extremes + listed points only',
            'applied_to_mpc': False}


class FootprintView(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, padding=10)
        self.app = app
        ttk.Label(self, wraplength=520, justify='left', text=(
            '실제 충돌 외형 (범퍼 포함). 기준: 구동축 중심, x 전방·y 좌측, 단위 m. 극값만으로 전체 외곽이 '
            '확인되지는 않으므로 필요하면 외곽점을 추가하세요. 결과는 기록만 하며 MPC에 자동 반영하지 '
            '않습니다.')).pack(fill='x')
        self.vars = {}
        for key, label, default in (('front', '전방 최대 (m)', ''), ('rear', '후방 최대 (m)', ''),
                                    ('left', '좌측 최대 (m)', ''), ('right', '우측 최대 (m)', ''),
                                    ('width', '범퍼 포함 전체 폭 (m)', ''),
                                    ('length', '범퍼 포함 전체 길이 (m)', ''),
                                    ('origin_offset_x', 'MPC 원점의 구동축 대비 x (m)', '0'),
                                    ('uncertainty', '측정 오차/반복 편차 (m)', ''),
                                    ('photo', '실측 사진 경로(선택)', '')):
            row = ttk.Frame(self)
            row.pack(fill='x', pady=1)
            ttk.Label(row, text=label, width=28).pack(side='left')
            self.vars[key] = tk.StringVar(value=default)
            ttk.Entry(row, textvariable=self.vars[key], width=24).pack(side='left')
        self.bumper = tk.BooleanVar(value=True)
        ttk.Checkbutton(self, text='범퍼 장착 상태', variable=self.bumper).pack(anchor='w')
        ttk.Label(self, text='추가 외곽점 "x,y; x,y" (m, 구동축 기준)').pack(anchor='w')
        self.points = tk.Text(self, height=3, width=60)
        self.points.pack(fill='x')
        row = ttk.Frame(self)
        row.pack(fill='x', pady=4)
        ttk.Button(row, text='미리보기', command=self.preview).pack(side='left')
        ttk.Button(row, text='측정값 저장 → 기록 추가',
                   command=lambda: app.job(app.save_footprint)).pack(side='left', padx=4)
        self.canvas = tk.Canvas(self, width=360, height=300, background='white')
        self.canvas.pack(pady=4)
        self.result = ttk.Label(self, text='', wraplength=520, justify='left')
        self.result.pack(fill='x')

    def inputs(self):
        out = {k: v.get().strip() for k, v in self.vars.items()}
        out.update(bumper=self.bumper.get(), points=self.points.get('1.0', 'end'))
        return out

    def preview(self):
        try:
            v = self.inputs()
            result = evaluate(*(float(v[k]) for k in ('front', 'rear', 'left', 'right')),
                              float(v['origin_offset_x'] or 0), parse_points(v['points']))
        except ValueError as error:
            self.result.configure(text=f'입력 오류: {error}', foreground='#b03a2e')
            return
        self.draw(result)

    def draw(self, result):
        c, scale, ox, oy = self.canvas, 1200., 180, 150   # px per m, origin

        def p(x, y):
            return ox - y * scale, oy - x * scale         # screen up = robot forward
        c.delete('all')
        c.create_line(*p(-.12, 0), *p(.12, 0), fill='#bbbbbb')
        c.create_line(*p(0, -.12), *p(0, .12), fill='#bbbbbb')
        c.create_text(*p(.115, 0), text='+x', fill='#777777')
        for x0, y0, x1, y1 in FOOTPRINT_RECTS:
            c.create_rectangle(*p(x0, y0), *p(x1, y1), outline='#2e86c1')
        for cx, cy, r in FOOTPRINT_DISCS:
            (ax, ay), (bx, by) = p(cx + r, cy + r), p(cx - r, cy - r)
            c.create_oval(min(ax, bx), min(ay, by), max(ax, bx), max(ay, by),
                          outline='#a569bd', dash=(3, 2))
        pts = result['points_mpc_frame']
        flat = [v for x, y in pts[:4] for v in p(x, y)]
        c.create_polygon(*flat, outline='#c0392b', fill='', width=2)
        for x, y in pts[4:]:
            sx, sy = p(x, y)
            c.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill='#c0392b')
        c.create_oval(ox - 3, oy - 3, ox + 3, oy + 3, fill='black')
        margin = result['disc_cover_min_margin_m']
        box = result['bbox_corner_min_margin_m']
        self.result.configure(foreground='#222222', text=(
            '파랑: URDF 상자 · 보라 점선: MPC 원 근사 · 빨강: 실측 극값 사각형/외곽점 · 검정: 원점\n'
            + (f"외곽점 기준 원 근사 최소 여유 {margin*1000:.1f} mm → "
               f"{'외곽점 모두 포함' if margin >= 0 else '외곽점이 원 밖에 있음'}"
               if margin is not None else '외곽점 미입력: 포함 여부 판정 안 함')
            + (f"\n(참고) 극값 사각형 모서리 최소 여유 {box*1000:.1f} mm — 모서리는 실제 표면점이 "
               '아닐 수 있음' if box is not None else '')
            + '\n전후좌우 극값(+추가점)만 확인한 결과이며 전체 외곽 검증이 아닙니다.'))
