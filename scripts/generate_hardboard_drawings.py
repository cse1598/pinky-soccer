#!/usr/bin/env python3
"""Generate dimensioned fabrication drawings and actual-size A4 templates from field YAML."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml
from pypdf import PdfWriter
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


MM = 72/25.4
PAGE = landscape(A4)
INK = colors.HexColor('#172b3a')
BLUE = colors.HexColor('#2369ad')
RED = colors.HexColor('#b43535')


def text(c, x, y, value, size=8, color=INK, align='left'):
    c.setFillColor(color)
    c.setFont('Korean', size)
    method = {'left': c.drawString, 'center': c.drawCentredString, 'right': c.drawRightString}[align]
    method(x*MM, y*MM, str(value))


def line(c, x1, y1, x2, y2, color=INK, width=.2):
    c.setStrokeColor(color)
    c.setLineWidth(width*MM)
    c.line(x1*MM, y1*MM, x2*MM, y2*MM)


def rect(c, x, y, w, h, fill=None, stroke=INK, width=.2):
    c.setLineWidth(width*MM)
    c.setStrokeColor(stroke)
    if fill is not None:
        c.setFillColor(fill)
    c.rect(x*MM, y*MM, w*MM, h*MM, stroke=1, fill=fill is not None)


def cross(c, x, y, size=2, color=RED):
    line(c, x-size, y, x+size, y, color)
    line(c, x, y-size, x, y+size, color)


def dim(c, a, b, label, vertical=False):
    x1, y1 = a
    x2, y2 = b
    line(c, x1, y1, x2, y2, BLUE, .15)
    if vertical:
        for y in (y1, y2):
            line(c, x1-1.5, y-1.5, x1+1.5, y+1.5, BLUE)
        c.saveState()
        c.translate((x1-2)*MM, (y1+y2)/2*MM)
        c.rotate(90)
        text(c, 0, 0, label, color=BLUE, align='center')
        c.restoreState()
    else:
        for x in (x1, x2):
            line(c, x-1.5, y1-1.5, x+1.5, y1+1.5, BLUE)
        text(c, (x1+x2)/2, y1+2, label, color=BLUE, align='center')


def heading(c, title, subtitle, page, scale):
    text(c, 15, 195, title, 15)
    text(c, 15, 187, subtitle, 8)
    line(c, 15, 182, 282, 182, BLUE, .4)
    text(c, 15, 9, 'PINKY SOCCER / HARDBOARD / 2026-11-21 / 단위 mm', 7)
    text(c, 282, 9, f'{page}  |  {scale}', 7, align='right')


def bar(c, x, y, length=100):
    line(c, x, y, x+length, y, INK, .4)
    for v in range(0, length+1, 10):
        line(c, x+v, y-1, x+v, y+1)
    text(c, x+length/2, y+3, f'출력 검사: 이 선의 길이 {length} mm', 7, align='center')


def table(c, x, y, widths, rows, row_height=9):
    for r, row in enumerate(rows):
        left = x
        for width, cell in zip(widths, row):
            rect(c, left, y-(r+1)*row_height, width, row_height,
                 colors.HexColor('#e9f0f5') if r == 0 else colors.white,
                 colors.HexColor('#a5b2bc'), .12)
            text(c, left+2, y-r*row_height-row_height*.68, cell, 8)
            left += width


def overview(c, d):
    heading(c, '제2 경기장 - 전체 배치도', '판 610 x 910 네 장 / 외벽 안쪽 기준 / 원점은 경기장 중앙',
            '01', '1:10, A4 가로')
    s, x0, y0 = .1, 24., 42.
    length, width = d['total_length'], d['total_width']

    def xy(x, y):
        return x0+(x+length/2)*s, y0+(y+width/2)*s

    rect(c, x0, y0, length*s, width*s, colors.HexColor('#f4f7f2'))
    c.setDash(3, 3)
    line(c, *xy(0, -width/2), *xy(0, width/2), colors.grey, .12)
    line(c, *xy(-length/2, 0), *xy(length/2, 0), colors.grey, .12)
    c.setDash()
    lx, wy, lw = d['playing_length'], d['playing_width'], d['line_width']
    # Actual band outlines. All white boundary paint lies inside this outer rectangle.
    ox, oy = xy(-lx/2, -wy/2)
    rect(c, ox, oy, lx*s, wy*s, colors.HexColor('#bbbbbb'), INK, .08)
    rect(c, ox+lw*s, oy+lw*s, (lx-2*lw)*s, (wy-2*lw)*s,
         colors.HexColor('#f4f7f2'), INK, .08)
    for sign in (-1, 1):
        front = sign*lx/2
        gx, gy = xy(min(front, front+sign*d['goal_depth']), -d['goal_width']/2)
        rect(c, gx, gy, d['goal_depth']*s, d['goal_width']*s,
             colors.HexColor('#d9e7f6') if sign == 1 else colors.HexColor('#f6edba'))
        radius, cy = d['penalty_radius'], d['penalty_width']/2-d['penalty_radius']
        cx = lx/2-d['penalty_depth']+radius
        paths = []
        for side in (-1, 1):
            start = side*math.pi/2
            points = [(sign*lx/2, side*(d['penalty_width']/2-lw/2)),
                      (sign*cx, side*(d['penalty_width']/2-lw/2))]
            points += [(sign*(cx+(radius-lw/2)*math.cos(start+side*i*math.pi/2/64)),
                        side*cy+(radius-lw/2)*math.sin(start+side*i*math.pi/2/64))
                       for i in range(65)]
            paths.append(points)
        paths.append([(sign*(lx/2-d['penalty_depth']+lw/2), -cy),
                      (sign*(lx/2-d['penalty_depth']+lw/2), cy)])
        c.setLineWidth(lw*s*MM)
        c.setStrokeColor(colors.HexColor('#aaaaaa'))
        for points in paths:
            p = c.beginPath()
            p.moveTo(*(v*MM for v in xy(*points[0])))
            for point in points[1:]:
                p.lineTo(*(v*MM for v in xy(*point)))
            c.drawPath(p)
    cx, cy = xy(0, 0)
    c.setStrokeColor(INK)
    c.setLineWidth(d['thin_black_line']*s*MM)
    c.circle(cx*MM, cy*MM, d['center_diameter']/2*s*MM)
    for x, y in [(0, 0)]+[(sx*(lx/2-d['neutral_distance_from_end']), sy*d['penalty_width']/2)
                          for sx in (-1, 1) for sy in (-1, 1)]:
        px, py = xy(x, y)
        c.setFillColor(INK)
        c.circle(px*MM, py*MM, d['neutral_diameter']/2*s*MM, fill=1, stroke=0)
    cross(c, cx, cy, 2)
    dim(c, (x0, 174), (x0+length*s, 174), f'{length:g} (외벽 안쪽)')
    dim(c, (14, y0), (14, y0+width*s), f'{width:g}', True)
    dim(c, (ox, 31), (ox+lx*s, 31), f'{lx:g} (흰 선 바깥 가장자리)')
    text(c, cx, 20, '점선은 판 이음매입니다. 중앙 가로/세로 선을 추가로 칠하지 않습니다.', 8, align='center')
    text(c, 218, 170, '제작 기준', 11)
    notes = [f'외벽 높이: {d["wall_height"]:g}',
             f'골대 내부: {d["goal_width"]:g} x {d["goal_height"]:g} x {d["goal_depth"]:g}',
             f'골대 판 두께: {d["goal_thickness"]:g}', f'흰 선 폭: {lw:g}',
             f'바깥 여백: 네 방향 {d["outer_strip"]:g}',
             f'중앙 원 선중심 지름: {d["center_diameter"]:g}',
             f'검정 원 선 폭: {d["thin_black_line"]:g}',
             f'중립점 지름: {d["neutral_diameter"]:g}', '회색 띠 = 흰 선 칠할 곳', '빨간 십자 = 위치 기준',
             '오른쪽 = 파란 골대 (+X)', '왼쪽 = 노란 골대 (-X)']
    for i, note in enumerate(notes):
        text(c, 218, 159-i*10, note, 8)
    c.showPage()


def coordinates(c, d):
    heading(c, '라인 배치 - 줄자로 옮길 좌표', '기준: 왼쪽 아래 외벽 안쪽 모서리 A=(0,0), 오른쪽 X / 위쪽 Y',
            '02', '치수표 / 축척 없음')
    l, w, o = d['total_length'], d['total_width'], d['outer_strip']
    gw, pw, pd, pr = (d[k] for k in ('goal_width', 'penalty_width', 'penalty_depth', 'penalty_radius'))
    gy = f'{w/2-gw/2:g}, {w/2+gw/2:g}'
    py = f'{w/2-pw/2:g}, {w/2+pw/2:g}'
    arc_y = f'{w/2-pw/2+pr:g}, {w/2+pw/2-pr:g}'
    rows = [['위치 / 선', 'A 기준 X', 'A 기준 Y', '그리는 기준']]
    rows += [['흰 경계선 바깥 네 모서리', f'{o:g}, {l-o:g}', f'{o:g}, {w-o:g}', f'사각형 안쪽으로 {d["line_width"]:g} 폭'],
             ['경기장 중앙 / 중앙 원 중심', f'{l/2:g}', f'{w/2:g}', f'원 선중심 반지름 {d["center_diameter"]/2:g}'],
             ['골대 입구 중앙 (좌 / 우)', f'{o:g}, {l-o:g}', f'{w/2:g}', f'폭 {gw:g}, 높이 {d["goal_height"]:g}'],
             ['골대 입구 Y 끝', '좌/우 골라인', gy, '기둥 안쪽이 이 좌표'],
             ['페널티 구역 위/아래 외곽', f'각 골라인에서 {pd:g} 안쪽', py, f'바깥 폭 {pw:g}'],
             ['페널티 앞선 바깥 가장자리', f'{o+pd:g}, {l-o-pd:g}', arc_y, f'양끝 R{pr:g} 코너 연결'],
             ['페널티 코너 원 중심 X', f'{o+pd-pr:g}, {l-o-pd+pr:g}', '아래 행의 Y 사용', f'바깥 R{pr:g}, 안쪽 R{pr-d["line_width"]:g}'],
             ['페널티 코너 원 중심 Y', '위 행의 X 사용', arc_y, '네 코너 동일 / 좌우 반전'],
             ['중립점 4개 중심', f'{o+d["neutral_distance_from_end"]:g}, {l-o-d["neutral_distance_from_end"]:g}',
              py, f'각 조합에 지름 {d["neutral_diameter"]:g} 검정점']]
    table(c, 15, 171, [66, 65, 44, 92], rows, 10)
    instructions = ['1. 판을 2 x 2로 놓고 안쪽 가로 1820, 세로 1220인지 먼저 확인합니다.',
                    '2. A에서 80 떨어진 사각형을 잡고 흰 선 15mm를 사각형 안쪽으로 칠합니다.',
                    '3. 페널티는 바깥 폭 540 / 깊이 170 / 바깥 코너 R100 기준입니다.',
                    '4. 중앙 원은 선중심 반지름 200, 검정 폭 2입니다. 바깥 지름은 402입니다.',
                    '5. 판 이음매는 테이프 등으로 평평하게 연결하고, 도면 점선은 칠하지 않습니다.']
    for i, value in enumerate(instructions):
        text(c, 18, 55-i*8, value, 9)
    c.showPage()


def goal(c, d):
    heading(c, '골대 - 5mm 판 맞댐 조립', '노랑/파랑 동일 치수, 각 1개 / 바닥판 없음 / 내부 공간 기준',
            '03', '조립도 1:3 / 재단표 실치수')
    width, height, depth, t = (d[k] for k in ('goal_width', 'goal_height', 'goal_depth', 'goal_thickness'))
    s, x, y = 1/3, 28., 135.
    rect(c, x-t*s, y, (width+2*t)*s, (height+t)*s, colors.HexColor('#dceafa'))
    rect(c, x, y, width*s, height*s, colors.white)
    dim(c, (x, y-8), (x+width*s, y-8), f'{width:g} 내부 폭')
    dim(c, (x+width*s+8, y), (x+width*s+8, y+height*s), f'{height:g}', True)
    text(c, x, 171, '정면: 지붕 아래가 높이 70, 기둥 안쪽 사이가 폭 400', 8)
    side_x = 210
    rect(c, side_x, y, (depth+t)*s, height*s, colors.HexColor('#dceafa'))
    rect(c, side_x, y+height*s, (depth+t)*s, t*s, colors.HexColor('#c3d9ec'))
    line(c, side_x+depth*s, y, side_x+depth*s, y+height*s, RED)
    dim(c, (side_x, y-8), (side_x+(depth+t)*s, y-8), f'{depth+t:g} 외형 깊이')
    text(c, side_x, 171, '측면: 내부 깊이 50 + 뒷판 5', 8)
    text(c, 20, 110, '재단 목록 - 모두 5mm 판, 아래 수량은 골대 2개 합계', 11)
    table(c, 20, 103, [35, 69, 30, 120], [
        ['부품', '가로 x 세로 (mm)', '총 수량', '조립 위치'],
        ['A 뒷판', f'{width:g} x {height:g}', '2장', '옆판 두 장 사이에 넣기'],
        ['B 옆판', f'{depth+t:g} x {height:g}', '4장', '앞끝부터 뒷판 바깥끝까지 55'],
        ['C 지붕', f'{width+2*t:g} x {depth+t:g}', '2장', 'A/B 윗면에 얹기, 바깥 폭 410'],
    ], 10)
    notes = ['조립 순서: 옆판 B 두 장을 안쪽 간격 400으로 세움 -> 뒤에 A를 사이에 끼움 -> C를 위에 얹음.',
             '지붕을 옆판 사이로 끼우지 않습니다. 지붕 아래 높이 70을 유지해야 합니다.',
             '파란 골대 앞끝은 A기준 X=1740, 노란 골대 앞끝은 X=80에 맞춥니다. 개구부 Y=410~810.',
             '부품 외곽 재단선 기준입니다. 칼날은 버릴 쪽으로 대고, 모형 조립 후 안쪽 400/70/50을 확인하세요.']
    for i, value in enumerate(notes):
        text(c, 20, 53-i*9, value, 8)
    c.showPage()


def corner(c, d):
    heading(c, '라인 제작 - 페널티 코너 실물 템플릿', 'A4 가로 / 실제 크기 100% / 페이지에 맞추기 OFF',
            '04', '1:1')
    cx, cy, radius = 132., 58., d['penalty_radius']
    lw = d['line_width']
    c.setStrokeColor(colors.HexColor('#bbbbbb'))
    c.setLineWidth(lw*MM)
    c.arc((cx-radius+lw/2)*MM, (cy-radius+lw/2)*MM,
          (cx+radius-lw/2)*MM, (cy+radius-lw/2)*MM, 90, 90)
    c.setLineWidth(.2*MM)
    c.setStrokeColor(INK)
    for r in (radius, radius-lw):
        c.arc((cx-r)*MM, (cy-r)*MM, (cx+r)*MM, (cy+r)*MM, 90, 90)
    cross(c, cx, cy, 3)
    c.setDash(3, 3)
    line(c, cx-radius, cy, cx+8, cy, RED, .15)
    line(c, cx, cy-8, cx, cy+radius, RED, .15)
    c.setDash()
    text(c, 22, 166, '회색 영역을 흰색으로 칠합니다.', 9)
    text(c, 145, 150, '원 중심: 빨간 십자', 10)
    for i, value in enumerate(['바깥 반지름 100', '안쪽 반지름 85', '칠할 띠 폭 15',
                               '선중심 반지름 92.5', '한 장을 뒤집거나 돌려 네 코너에 사용']):
        text(c, 145, 136-i*10, value, 9)
    rect(c, 225, 51, 15, 60, colors.HexColor('#bbbbbb'))
    text(c, 217, 43, '흰 선 폭 15', 8)
    c.setFillColor(INK)
    c.circle(190*MM, 64*MM, d['neutral_diameter']/2*MM, stroke=0, fill=1)
    text(c, 177, 52, '검정 중립점 Ø7', 8)
    bar(c, 28, 25)
    text(c, 145, 25, '인쇄 후 100mm 선을 자로 확인하고 사용하세요.', 8)
    c.showPage()


def tile_base(c, title, subtitle, page):
    heading(c, title, subtitle, page, '1:1 / A4 가로 / 실제 크기 100%')


def goal_templates(path, d):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    c.setTitle('Hardboard goal panel cutting templates - 5mm')
    width, height, depth, t = (d[k] for k in ('goal_width', 'goal_height', 'goal_depth', 'goal_thickness'))
    for col in range(2):
        start = col*210
        tile_base(c, f'골대 A 뒷판 + C 지붕 - {col+1}/2',
                  '두 장을 겹침 십자로 연결 / 파란 사각형은 종이 절단선, 검정선만 부품 재단', f'G{col+1}')
        c.saveState()
        clip = c.beginPath();clip.rect(20*MM, 28*MM, 230*MM, 145*MM)
        c.clipPath(clip, stroke=0)
        for name, y, w, h in [('A 뒷판 (총 2개)', 28., width, height),
                               ('C 지붕 (총 2개)', 113., width+2*t, depth+t)]:
            rect(c, 20-start, y, w, h)
            text(c, 25, y+10, f'{name} / 완성 치수 {w:g} x {h:g}', 8)
        for gx in (215., 225.):
            for gy in (106., 171.):
                cross(c, 20+gx-start, gy, 2)
        c.restoreState()
        rect(c, 20, 28, 230, 145, stroke=BLUE, width=.12)
        text(c, 255, 150, '좌표 범위', 7)
        text(c, 255, 141, f'X {start}~{start+230}', 7)
        text(c, 255, 125, '겹침 20mm', 7)
        bar(c, 25, 19)
        c.showPage()
    tile_base(c, '골대 B 옆판 - 실물 크기 4개', '골대 2개 합계 / 모두 5mm 판 / 55 x 70mm', 'G3')
    for i in range(4):
        x, y = 35+(i % 2)*80, 26+(i//2)*80
        rect(c, x, y, depth+t, height)
        text(c, x+5, y+8, f'B{i+1} / 55 x 70', 8)
    text(c, 200, 150, '검정 외곽선을 따라 재단', 9)
    text(c, 200, 138, '5mm 판에 본을 옮겨 사용', 8)
    text(c, 200, 126, '지붕은 윗면에 얹습니다.', 8)
    bar(c, 190, 28, 80)
    c.showPage()
    c.save()


def circle_templates(path, d):
    c = canvas.Canvas(str(path), pagesize=PAGE)
    c.setTitle('Hardboard field centre circle - actual size tiled A4')
    # 2 columns x 3 rows: 230 x 145 mm viewports with a 15 mm overlap.
    for row in range(3):
        for col in range(2):
            x0, y0 = col*215., (2-row)*130.
            tile_base(c, f'중앙 원 - {row+1}행 {col+1}열 / 3행 x 2열',
                      '출력 100% / 선중심 Ø400, 검정 선 폭 2 / 빨간 십자는 종이 정렬용',
                      f'C{row+1}{col+1}')
            c.saveState()
            clip = c.beginPath();clip.rect(20*MM, 28*MM, 230*MM, 145*MM)
            c.clipPath(clip, stroke=0)
            cx, cy = 20+201-x0, 28+201-y0
            c.setLineWidth(d['thin_black_line']*MM)
            c.setStrokeColor(INK)
            c.circle(cx*MM, cy*MM, d['center_diameter']/2*MM)
            cross(c, cx, cy, 3)
            if col == 0 and row == 1:
                text(c, cx-4, cy+5, '원 중심', 8, color=RED, align='right')
            for x in (219., 226.):
                for y in (134., 141., 264., 271.):
                    cross(c, 20+x-x0, 28+y-y0, 1.5)
            for x in (50., 150., 300., 390.):
                for y in (134., 141., 264., 271.):
                    cross(c, 20+x-x0, 28+y-y0, 1.5)
            c.restoreState()
            rect(c, 20, 28, 230, 145, stroke=BLUE, width=.12)
            text(c, 255, 153, '배치표', 8)
            for j, label in enumerate(('11  12', '21  22', '31  32')):
                text(c, 255, 142-j*9, label, 8)
            text(c, 255, 100, '15mm 겹침', 7)
            text(c, 255, 88, '같은 십자를', 7)
            text(c, 255, 80, '포개어 부착', 7)
            bar(c, 25, 19)
            c.showPage()
    c.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', default='src/pinky_soccer_sim/config/hardboard_field.yaml')
    parser.add_argument('--output', default='output/pdf')
    parser.add_argument('--font', required=True, help='Korean TrueType font to embed')
    args = parser.parse_args()
    spec = Path(args.spec)
    cfg = yaml.safe_load(spec.read_text())
    d = {k: v*1000 for k, v in cfg.items() if isinstance(v, (float, int))}
    d['total_length'] = d['playing_length']+2*d['outer_strip']
    d['total_width'] = d['playing_width']+2*d['outer_strip']
    # These fabrication sheets are for the agreed hardboard design, not arbitrary RCJ fields.
    expected = {'total_length': 1820, 'total_width': 1220, 'goal_thickness': 5,
                'goal_width': 400, 'goal_height': 70, 'goal_depth': 50,
                'line_width': 15, 'center_diameter': 400, 'thin_black_line': 2,
                'penalty_width': 540, 'penalty_depth': 170, 'penalty_radius': 100,
                'neutral_diameter': 7, 'neutral_distance_from_end': 300, 'outer_strip': 80}
    if any(not math.isclose(d.get(k, float('nan')), v) for k, v in expected.items()):
        raise ValueError('Fabrication layout requires the agreed 1820x1220 field and 5mm goal stock')
    pdfmetrics.registerFont(TTFont('Korean', args.font))
    output = Path(args.output);output.mkdir(parents=True, exist_ok=True)
    parts = output/'parts';parts.mkdir(exist_ok=True)
    path = parts/'hardboard_field_fabrication.pdf'
    c = canvas.Canvas(str(path), pagesize=PAGE)
    c.setTitle('Pinky Soccer hardboard field fabrication drawings')
    overview(c, d);coordinates(c, d);goal(c, d);corner(c, d);c.save()
    goal_templates(parts/'hardboard_goal_cut_templates_A4.pdf', d)
    circle_templates(parts/'hardboard_center_circle_A4.pdf', d)
    writer = PdfWriter()
    for filename, title in [('hardboard_field_fabrication.pdf', '1-4 Field layout and assembly'),
                             ('hardboard_goal_cut_templates_A4.pdf', '5-7 Goal cutting templates'),
                             ('hardboard_center_circle_A4.pdf', '8-13 Centre circle template')]:
        writer.append(parts/filename, outline_item=title)
    final = output/'hardboard_fabrication_pack.pdf'
    with final.open('wb') as stream:
        writer.write(stream)
    writer.close()
    manifest = {'source_yaml': str(spec), 'source_sha256': hashlib.sha256(spec.read_bytes()).hexdigest(),
                'units': 'mm', 'goal_stock_mm': d['goal_thickness'],
                'inside_floor_mm': [d['total_length'], d['total_width']],
                'goal_panels_for_two_goals': [
                    {'part': 'back', 'width_mm': 400, 'height_mm': 70, 'quantity': 2},
                    {'part': 'side', 'width_mm': 55, 'height_mm': 70, 'quantity': 4},
                    {'part': 'roof', 'width_mm': 410, 'height_mm': 55, 'quantity': 2}],
                'circle_tiles': {'columns': 2, 'rows': 3, 'overlap_mm': 15},
                'print_scale': '100%, no fit to page'}
    (output/'hardboard_drawing_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(final)


if __name__ == '__main__':
    main()
