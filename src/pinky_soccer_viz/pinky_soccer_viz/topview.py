"""OpenCV top-view renderer for /world_model (pure function, testable without ROS)."""

import math

import cv2
import numpy as np
from pinky_soccer_viz import style
from pinky_soccer_viz.style import bgr, ESTIMATED, LOST, OBSERVED

PANEL_WIDTH = 300
MARGIN = 0.08   # [m] around the outer walls


def stamp_sec(t):
    """Return builtin_interfaces/Time as float seconds."""
    return t.sec + t.nanosec * 1e-9


class TopView:
    """Draws the field once and overlays the dynamic world state each frame."""

    def __init__(self, field, px_per_m=200.0, opponent_arrows=None):
        self.field = field
        self.opponent_arrows = opponent_arrows or {}
        self.s = px_per_m
        walls = field.inner_walls
        t = field.spec['wall_thickness']
        self.xmin, self.xmax = walls.xmin - t - MARGIN, walls.xmax + t + MARGIN
        self.ymin, self.ymax = walls.ymin - t - MARGIN, walls.ymax + t + MARGIN
        self.w = int(math.ceil((self.xmax - self.xmin) * self.s))
        self.h = int(math.ceil((self.ymax - self.ymin) * self.s))
        self.background = self._draw_field()

    # ---------------------------------------------------------------- helpers
    def px(self, x, y):
        """Map coordinates -> integer pixel (u right, v down)."""
        return (int(round((x - self.xmin) * self.s)), int(round((self.ymax - y) * self.s)))

    def m(self, length):
        """Metres -> pixels."""
        return max(1, int(round(length * self.s)))

    def _rect(self, img, rect, color, thickness):
        cv2.rectangle(img, self.px(rect.xmin, rect.ymax), self.px(rect.xmax, rect.ymin),
                      color, thickness)

    def _draw_field(self):
        f = self.field
        img = np.full((self.h, self.w, 3), 60, np.uint8)
        t = f.spec['wall_thickness']
        walls = f.inner_walls
        cv2.rectangle(img, self.px(walls.xmin - t, walls.ymax + t),
                      self.px(walls.xmax + t, walls.ymin - t), bgr('wall'), -1)
        self._rect(img, walls, bgr('carpet'), -1)
        for color in ('yellow', 'blue'):
            self._rect(img, f.goal_interior(color), bgr(color), -1)
        lw = max(1, self.m(f.spec['line_width']))
        self._rect(img, f.playing_area, bgr('line'), lw)
        for color in ('yellow', 'blue'):
            self._rect(img, f.penalty_area(color), bgr('line'), lw)
            (x0, y0), (x1, y1) = f.goal_mouth(color)
            cv2.line(img, self.px(x0, y0), self.px(x1, y1), bgr(color), lw + 1)
        cx, cy, r = f.center_circle
        cv2.circle(img, self.px(cx, cy), self.m(r), bgr('center'), 1, cv2.LINE_AA)
        for x, y in f.neutral_spots:
            cv2.circle(img, self.px(x, y), 2, bgr('center'), -1)
        return img

    def _blend(self, img, overlay, alpha):
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    def _ellipse(self, img, center, cov, color):
        major, minor, angle = style.covariance_ellipse(cov)
        if major <= 0:
            return
        cv2.ellipse(img, self.px(*center), (self.m(major), self.m(minor)),
                    -math.degrees(angle), 0, 360, color, 1, cv2.LINE_AA)

    def _dotted(self, img, pts, color):
        for a, b in zip(pts[::2], pts[1::2]):
            cv2.line(img, self.px(*a), self.px(*b), color, 1, cv2.LINE_AA)

    # ---------------------------------------------------------------- frame
    def render(self, wm=None, goal=None, truth=None, hz=0.0, hfov=math.radians(60),
               fov_range=1.5):
        """Return a BGR image of the field, world state and status panel."""
        img = self.background.copy()
        truth = truth or {}
        if wm is not None:
            self._draw_grid(img, wm.ball_search_grid)
            me = (wm.self.pose.x, wm.self.pose.y, wm.self.pose.theta)
            overlay = img.copy()
            poly = np.array([self.px(x, y) for x, y in style.fov_polygon(me, hfov, fov_range)])
            cv2.fillPoly(overlay, [poly], bgr('fov'))
            self._blend(img, overlay, 0.15)
        for name in ('robot', 'ball'):
            if name in truth:
                cv2.circle(img, self.px(*truth[name]), 4, bgr('truth'), -1, cv2.LINE_AA)
        if goal is not None:
            self._draw_goal(img, goal)
        if wm is not None:
            now = stamp_sec(wm.header.stamp)
            for opp in wm.opponents:
                self._draw_opponent(img, opp, now)
            self._draw_ball(img, wm.ball)
            self._draw_self(img, wm.self)
        panel = self._panel(wm, goal, hz, truth)
        if panel.shape[0] > img.shape[0]:
            pad = np.full((panel.shape[0] - img.shape[0], img.shape[1], 3), 30, np.uint8)
            img = np.vstack([img, pad])
        return np.hstack([img, panel])

    def _draw_grid(self, img, grid):
        info = grid.info
        if info.width == 0 or info.height == 0:
            return
        data = np.array(grid.data, np.int16).reshape(info.height, info.width)
        heat = (np.clip(data, 0, 100) * 255 // 100).astype(np.uint8)   # no uint8 overflow
        colored = cv2.applyColorMap(np.flipud(heat).astype(np.uint8), cv2.COLORMAP_INFERNO)
        x0, y0 = info.origin.position.x, info.origin.position.y
        u0, v1 = self.px(x0, y0)
        u1, v0 = self.px(x0 + info.width * info.resolution, y0 + info.height * info.resolution)
        u0, v0 = max(u0, 0), max(v0, 0)
        u1, v1 = min(u1, self.w), min(v1, self.h)
        if u1 <= u0 or v1 <= v0:
            return
        patch = cv2.resize(colored, (u1 - u0, v1 - v0), interpolation=cv2.INTER_NEAREST)
        img[v0:v1, u0:u1] = cv2.addWeighted(patch, 0.45, img[v0:v1, u0:u1], 0.55, 0)

    def _draw_self(self, img, obj):
        x, y, th = obj.pose.x, obj.pose.y, obj.pose.theta
        c = self.px(x, y)
        cv2.circle(img, c, self.m(style.ROBOT_RADIUS), bgr('self'), 2, cv2.LINE_AA)
        tip = self.px(x + 0.12 * math.cos(th), y + 0.12 * math.sin(th))
        cv2.arrowedLine(img, c, tip, bgr('self'), 2, cv2.LINE_AA, tipLength=0.3)
        if obj.state == ESTIMATED:
            self._ellipse(img, (x, y), obj.covariance, bgr('self'))

    def _draw_ball(self, img, obj):
        x, y = obj.pose.x, obj.pose.y
        r = max(3, self.m(0.02135))
        if obj.state == OBSERVED:
            cv2.circle(img, self.px(x, y), r, bgr('ball'), -1, cv2.LINE_AA)
        elif obj.state == ESTIMATED:
            overlay = img.copy()
            cv2.circle(overlay, self.px(x, y), r, bgr('ball'), -1, cv2.LINE_AA)
            self._blend(img, overlay, style.FAINT_ALPHA)
            self._ellipse(img, (x, y), obj.covariance, bgr('ball'))
        else:
            u, v = self.px(x, y)
            d = r + 2
            cv2.line(img, (u - d, v - d), (u + d, v + d), bgr('ball'), 2, cv2.LINE_AA)
            cv2.line(img, (u - d, v + d), (u + d, v - d), bgr('ball'), 2, cv2.LINE_AA)
            return
        if math.hypot(obj.velocity.x, obj.velocity.y) > 0.02:
            pts = style.ball_prediction((x, y), (obj.velocity.x, obj.velocity.y), steps=12)
            self._dotted(img, pts, bgr('ball'))

    def _draw_opponent(self, img, obj, now):
        if obj.state == LOST:
            return
        centre = self.px(obj.pose.x, obj.pose.y)
        radius = self.m(style.OPPONENT_RADIUS)
        overlay = img.copy()
        cv2.circle(overlay, centre, radius, bgr('opponent'), -1, cv2.LINE_AA)
        self._blend(img, overlay, 1. if obj.state == OBSERVED else style.FAINT_ALPHA)
        vector = style.opponent_velocity(obj, **self.opponent_arrows)
        if vector is not None:
            tip = self.px(obj.pose.x+vector[0], obj.pose.y+vector[1])
            cv2.arrowedLine(img, centre, tip, bgr('opponent_velocity'), 2,
                            cv2.LINE_AA, tipLength=.25)
        if obj.state != OBSERVED:
            age = max(0.0, now-stamp_sec(obj.last_seen))
            cv2.putText(img, f'{age:.1f}s', (centre[0], centre[1]-radius-3),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, bgr('opponent'), 1, cv2.LINE_AA)

    def _draw_goal(self, img, goal):
        if goal.mode == 2:      # STOP
            return
        t = goal.target
        c = self.px(t.x, t.y)
        cv2.drawMarker(img, c, bgr('target'), cv2.MARKER_CROSS, 14, 2)
        tip = self.px(t.x + 0.08 * math.cos(t.theta), t.y + 0.08 * math.sin(t.theta))
        cv2.arrowedLine(img, c, tip, bgr('target'), 1, cv2.LINE_AA, tipLength=0.3)
        pts = [self.px(p.pose.position.x, p.pose.position.y) for p in goal.planned_path.poses]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts)], False, bgr('path'), 1, cv2.LINE_AA)

    def _panel(self, wm, goal, hz, truth):
        lines = [('Pinky Soccer world model', (255, 255, 255))]
        if goal is not None:
            lines += [(f'branch : {goal.branch}', (255, 200, 255)),
                      (f'action : {goal.action}', (255, 200, 255))]
        else:
            lines.append(('no /behavior_goal', (160, 160, 160)))
        if wm is not None:
            ball = wm.ball
            major, _, _ = style.covariance_ellipse(ball.covariance)
            since = max(0.0, stamp_sec(wm.header.stamp) - stamp_sec(ball.last_seen))
            lines += [
                (f'playing: {wm.playing}  ours: {wm.our_goal_color}', (220, 220, 220)),
                (f'ball   : {style.STATE_NAMES.get(ball.state, "?")}', bgr('ball')),
                (f'ball 2s major axis: {2 * major:.2f} m', bgr('ball')),
                (f'since seen: {since:.1f} s', bgr('ball')),
                (f'ball (x,y): {ball.pose.x:+.2f} {ball.pose.y:+.2f}', bgr('ball')),
                (f'self (x,y,th): {wm.self.pose.x:+.2f} {wm.self.pose.y:+.2f} '
                 f'{wm.self.pose.theta:+.2f}', bgr('self')),
                (f'opponents: {sum(o.state != LOST for o in wm.opponents)}', bgr('opponent')),
            ]
        else:
            lines.append(('no /world_model', (160, 160, 160)))
        lines.append((f'/world_model: {hz:.1f} Hz', (220, 220, 220)))
        if truth:
            lines.append(('grey dots: sim ground truth', bgr('truth')))
        legend = ['legend: solid=observed', 'faint+ellipse(2s)=estimated', 'X=lost ball',
                  'dots=1.5 s prediction', '+ magenta=behavior target']
        height = max(self.h, 24 + 22 * len(lines) + 18 * len(legend) + 20)
        panel = np.full((height, PANEL_WIDTH, 3), 30, np.uint8)
        for i, (text, color) in enumerate(lines):
            cv2.putText(panel, text, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color,
                        1, cv2.LINE_AA)
        for i, text in enumerate(legend):
            cv2.putText(panel, text, (10, height - 14 - 18 * (len(legend) - 1 - i)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 170, 170), 1, cv2.LINE_AA)
        return panel
