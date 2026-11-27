"""Geometry, time scaling and finite-horizon TVLQR; no ROS dependencies."""
from dataclasses import dataclass
import math
import numpy as np

DT = 0.02


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Limits:
    radius_l: float
    radius_r: float
    track: float
    wheel: float
    accel: float
    v: float
    w: float
    driver_r: float
    driver_b: float
    driver_v: float
    driver_w: float

    def wheels(self, u):
        v, w = np.asarray(u).T
        return np.stack(((v-w*self.track/2)/self.radius_l,
                         (v+w*self.track/2)/self.radius_r), axis=-1)

    def twist(self, wheels):
        l, r = np.asarray(wheels).T
        return np.stack(((self.radius_l*l+self.radius_r*r)/2,
                         (self.radius_r*r-self.radius_l*l)/self.track), axis=-1)

    def scale(self, u):
        wheel = self.wheels(u)
        l, r = wheel.T
        dv = self.driver_r*(l+r)/2
        dw = self.driver_r*(r-l)/self.driver_b
        return max(1., float(np.max(np.abs(wheel)))/self.wheel,
                   float(np.max(np.abs(np.asarray(u)[..., 0])))/self.v,
                   float(np.max(np.abs(np.asarray(u)[..., 1])))/self.w,
                   float(np.max(np.abs(dv)))/self.driver_v,
                   float(np.max(np.abs(dw)))/self.driver_w)

    def command(self, u, previous, dt):
        target = self.wheels(np.asarray(u)/self.scale(u))
        delta = target-previous
        return previous+delta/max(1., float(np.max(np.abs(delta)))/(self.accel*dt))


@dataclass
class Plan:
    states: np.ndarray
    inputs: np.ndarray
    gains: np.ndarray
    kind: str
    direction: int
    dt: float = DT

    @property
    def duration(self):
        return len(self.inputs)*self.dt


def segment(points, headings, sign, lim, dt):
    """Arc-length sampling (or pure rotation) with a rest-to-rest time law."""
    points = np.asarray(points, dtype=float)
    headings = np.unwrap(headings)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    angle = abs(headings[-1]-headings[0])
    length = arc[-1]
    if length < 1e-8 and angle < 1e-8:
        return None
    if length < 1e-8:
        arc = np.linspace(0., 1., len(points))
        duration = max(.12, 1.875*angle/lim.w)
    else:
        if np.any(np.diff(arc) < 1e-9):
            raise ValueError('degenerate curve')
        duration = max(.12, 1.875*length/lim.v)
    for _ in range(12):
        n = max(6, math.ceil(duration/dt))
        if n > 3000:
            raise ValueError('trajectory too long')
        t = np.linspace(0., 1., n+1)
        s = (10*t**3-15*t**4+6*t**5)*arc[-1]
        x = np.column_stack([np.interp(s, arc, p) for p in
                             (points[:, 0], points[:, 1], headings)])
        d = np.diff(x, axis=0)
        u = np.column_stack((sign*np.linalg.norm(d[:, :2], axis=1)/dt, d[:, 2]/dt))
        wheel = lim.wheels(u)
        acc = np.max(np.abs(np.diff(np.vstack((np.zeros(2), wheel, np.zeros(2))), axis=0)))/dt
        scale = max(lim.scale(u), math.sqrt(acc/lim.accel))
        if scale <= 1.000001:
            return x, u
        duration = n*dt*scale*1.025
    raise ValueError('time scaling failed')


def straight(start, goal, sign, lim, dt):
    d = goal[:2]-start[:2]
    heading = start[2] if np.linalg.norm(d) < 1e-8 else math.atan2(d[1], d[0])+(math.pi if sign < 0 else 0.)
    heading = start[2]+wrap(heading-start[2])
    final = heading+wrap(goal[2]-heading)
    return join([segment([start[:2]]*2, [start[2], heading], 0, lim, dt),
                 segment([start[:2], goal[:2]], [heading]*2, sign, lim, dt),
                 segment([goal[:2]]*2, [heading, final], 0, lim, dt)], start)


def join(segments, start):
    parts = [p for p in segments if p is not None]
    if not parts:
        return np.vstack((start, start)), np.zeros((1, 2))
    return np.vstack([parts[0][0]]+[p[0][1:] for p in parts[1:]]), np.vstack([p[1] for p in parts])


def curve(start, goal, sign, kind, tangent, lim, dt):
    delta = goal[:2]-start[:2]
    length = np.linalg.norm(delta)
    if length < .015:
        raise ValueError('short curve')
    a = sign*tangent*length*np.array([math.cos(start[2]), math.sin(start[2])])
    b = sign*tangent*length*np.array([math.cos(goal[2]), math.sin(goal[2])])
    t = np.linspace(0., 1., 401)[:, None]
    if kind == 'bezier':
        p = ((1-t)**3*start[:2]+3*(1-t)**2*t*(start[:2]+a/3)
             +3*(1-t)*t*t*(goal[:2]-b/3)+t**3*goal[:2])
        dp = (3*(1-t)**2*a/3+6*(1-t)*t*(delta-(a+b)/3)+3*t*t*b/3)
    else:
        c3, c4, c5 = 10*delta-6*a-4*b, -15*delta+8*a+7*b, 6*delta-3*a-3*b
        p = start[:2]+a*t+c3*t**3+c4*t**4+c5*t**5
        dp = a+3*c3*t**2+4*c4*t**3+5*c5*t**4
    if np.min(np.linalg.norm(dp, axis=1)) < .04*length:
        raise ValueError('near cusp')
    h = np.unwrap(np.arctan2(dp[:, 1], dp[:, 0])+(math.pi if sign < 0 else 0.))
    h += start[2]-h[0]
    ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    if np.max(np.abs(np.diff(h))/np.maximum(ds, 1e-9)) > 35.:
        raise ValueError('excess curvature')
    return segment(p, h, sign, lim, dt)


def gains(states, inputs, dt):
    """Backward finite-horizon Riccati recursion around the sampled reference."""
    q = np.diag([40., 40., 4.])*dt
    r = np.diag([1., .12])*dt
    p = np.diag([80., 80., 12.])
    result = np.zeros((len(inputs), 2, 3))
    for i in range(len(inputs)-1, -1, -1):
        th = states[i, 2]
        v = inputs[i, 0]
        c, s = math.cos(th), math.sin(th)
        a = np.eye(3)
        a[0, 2], a[1, 2] = -dt*v*s, dt*v*c
        b = dt*np.array([[c, 0.], [s, 0.], [0., 1.]])
        k = np.linalg.solve(r+b.T@p@b, b.T@p@a)
        result[i] = k
        p = q+a.T@p@(a-b@k)
        p = (p+p.T)/2
    return result


def plan(start, goal, lim, kind='bezier', dt=DT):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if kind not in ('straight', 'bezier', 'quintic') or not np.all(np.isfinite([start, goal])):
        raise ValueError('invalid planner or pose')
    candidates = []
    for sign in (1, -1):
        if kind != 'straight':
            for tangent in (.7, 1.3, 2.):
                try:
                    x, u = curve(start, goal, sign, kind, tangent, lim, dt)
                    candidates.append((len(u), x, u, kind, sign))
                except ValueError:
                    pass
        x, u = straight(start, goal, sign, lim, dt)
        candidates.append((len(u), x, u, 'straight', sign))
    # Prefer a feasible requested curve; straight is a documented fallback, not a hidden switch.
    preferred = [c for c in candidates if c[3] == kind] or candidates
    _, x, u, actual, sign = min(preferred, key=lambda c: c[0])
    return Plan(x, u, gains(x, u, dt), actual, sign, dt)


def track(plan, i, pose):
    error = np.asarray(pose)-plan.states[i]
    error[2] = wrap(error[2])
    return plan.inputs[i]-plan.gains[i]@error
