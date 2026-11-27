"""Small geometric path planner and calibrated pose follower with explicit travel direction.

No optimizer, command scheduling, or automatic forward/reverse recovery.
"""
import heapq
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def contact_speed_limit(pose, ball, velocity, maximum):
    """Reduce relative impact speed; a moving ball permits faster sustained pushing."""
    delta = np.asarray(ball)-np.asarray(pose[:2])
    distance = float(np.linalg.norm(delta))
    if distance >= .30:
        return maximum
    direction = delta/max(distance,1e-6)
    ball_forward = max(0.,float(np.dot(velocity,direction)))
    # Close with 6 cm/s relative speed at the bumper, blend to full speed away.
    return min(maximum, max(.06,ball_forward+.06+1.4*max(0.,distance-.11)))


class Grid:
    def __init__(self, bounds, radius, blocks=(), resolution=.04):
        self.bounds = bounds
        self.radius = radius
        self.blocks = blocks
        self.resolution = resolution
        self.points = np.empty((0, 2))
        self.circles = []

    def clear(self, point, extra=0.):
        x, y = point
        x0, y0, x1, y1 = self.bounds
        r = self.radius+extra
        if not (x0+r <= x <= x1-r and y0+r <= y <= y1-r):
            return False
        for a, b, c, d in self.blocks:
            if math.hypot(max(a-x, 0, x-c), max(b-y, 0, y-d)) < r:
                return False
        if any(math.hypot(x-cx,y-cy) < radius+extra for cx,cy,radius in self.circles):
            return False
        return not len(self.points) or np.min(np.linalg.norm(self.points-point, axis=1)) >= r

    def line(self, a, b):
        a, b = np.asarray(a), np.asarray(b)
        return all(self.clear(a+(b-a)*t) for t in np.linspace(0, 1,
            max(2, int(np.linalg.norm(b-a)/(self.resolution/2))+2)))

    def plan(self, start, target):
        start, target = np.asarray(start), np.asarray(target)
        if not self.clear(start) or not self.clear(target):
            return []
        if self.line(start, target):
            return [start, target]
        origin = np.array(self.bounds[:2])
        def key(p):
            return tuple(np.round((p-origin)/self.resolution).astype(int))
        def point(k):
            return origin+np.array(k)*self.resolution
        occupancy = {}
        def free(k):
            if k not in occupancy:
                occupancy[k] = self.clear(point(k), self.resolution/(2*math.sqrt(2)))
            return occupancy[k]
        snapped = key(start)
        # A valid continuous pose can round into an occupied grid cell. Connect
        # to a visible neighbouring cell instead of declaring the entire route absent.
        roots = [(snapped[0]+dx,snapped[1]+dy) for dx in (-1,0,1) for dy in (-1,0,1)]
        roots = [k for k in roots if free(k) and self.line(start,point(k))]
        if not roots:
            return []
        root = min(roots,key=lambda k:np.linalg.norm(point(k)-start))
        queue = [(0., root)]
        cost, parent = {root: 0.}, {}
        end = None
        closed = set()
        while queue:
            _, node = heapq.heappop(queue)
            if node in closed:
                continue
            closed.add(node)
            p = point(node)
            if np.linalg.norm(p-target) <= 2*self.resolution and self.line(p, target):
                end = node
                break
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (node[0]+dx, node[1]+dy)
                q = point(nxt)
                if not free(nxt):
                    continue
                if dx and dy and not (free((node[0]+dx,node[1])) and free((node[0],node[1]+dy))):
                    continue
                value = cost[node]+self.resolution*math.hypot(dx, dy)
                if value < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = value, node
                    heapq.heappush(queue, (value+np.linalg.norm(q-target), nxt))
        if end is None:
            return []
        path = [target]
        while end != root:
            path.append(point(end)); end = parent[end]
        path.extend([point(root), start]); path.reverse()
        # Greedy line-of-sight pruning; no unsafe curve rounding around obstacles.
        pruned = [path[0]]
        i = 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not self.line(path[i], path[j]):
                j -= 1
            pruned.append(path[j]); i = j
        return pruned


class Follower:
    def __init__(self, params, position=.035, heading=.09):
        self.p = params
        self.position, self.heading = position, heading
        self.reset()

    def reset(self):
        self.phase = 'TRAVEL'
        self.since = None
        self.done = False
        self.previous = np.zeros(2)

    def wheel_command(self, v, w, dt):
        p = self.p
        u = np.array([(v-p.m('track')*w/2)/p.m('left_radius'),
                      (v+p.m('track')*w/2)/p.m('right_radius')])
        u *= min(1., p.wheel_cmd_max()/max(max(abs(u)), 1e-9))
        self.previous += np.clip(u-self.previous, -p.wheel_accel_cmd_max()*dt,
                                p.wheel_accel_cmd_max()*dt)
        return self.previous.copy()

    def command(self, pose, target, waypoint, wheels, now, dt=.02, streaming=False,
                speed_cap=0., reverse=False):
        p = self.p
        measured_v = (p.m('left_radius')*wheels[0]+p.m('right_radius')*wheels[1])/2
        measured_w = (p.m('right_radius')*wheels[1]-p.m('left_radius')*wheels[0])/p.m('track')
        distance = math.hypot(target[0]-pose[0], target[1]-pose[1])
        angle = wrap(target[2]-pose[2])
        tau = max(p.m('tau_left'), p.m('tau_right'))
        # Calibrated lag + one watchdog period; speed-dependent braking envelope.
        lag = tau+.05+p.m('delay')
        accel = min(p.m('left_radius'), p.m('right_radius'))*p.wheel_accel_cmd_max()
        angular_accel = 2*accel/p.m('track')
        vmax = min(p.m('left_radius'), p.m('right_radius'))*p.wheel_cmd_max()
        wmax = 2*vmax/p.m('track')
        if speed_cap > 0:
            vmax = min(vmax, speed_cap)
        def speed(remaining, decel):
            return max(0., math.sqrt((decel*lag)**2+2*decel*max(0, remaining))-decel*lag)
        stopped = abs(measured_v)<.012 and abs(measured_w)<.08
        if self.done:
            return np.zeros(2)
        if distance <= self.position and self.phase == 'TRAVEL' and not streaming:
            self.phase = 'ALIGN'
        if self.phase == 'ALIGN':
            # Once position is reached do not drive back and forth to chase mm noise.
            if abs(angle) <= self.heading and abs(angle-measured_w*lag) <= self.heading:
                self.phase = 'SETTLE'
            else:
                desired = 2.5*angle-(.6+2.5*lag)*measured_w
                limit = min(wmax, speed(abs(angle), angular_accel))
                return self.wheel_command(0., max(-limit, min(limit, desired)), dt)
        if self.phase == 'SETTLE':
            self.previous[:] = 0
            accepted = stopped and abs(angle) <= self.heading and distance <= self.position+.025
            self.since = (now if self.since is None else self.since) if accepted else None
            self.done = self.since is not None and now-self.since >= .15
            return np.zeros(2)
        dx, dy = waypoint[0]-pose[0], waypoint[1]-pose[1]
        bearing = wrap(math.atan2(dy, dx)-pose[2]-(math.pi if reverse else 0.))
        # Direction is explicit; large heading errors rotate before translation.
        w = max(-wmax, min(wmax, 3.*bearing-0.25*measured_w))
        v = vmax*max(0., math.cos(bearing))**3
        if abs(bearing) > .65:
            v = 0.
        if not streaming:
            v = min(v, speed(max(0., distance-self.position/2), accel))
        return self.wheel_command(-v if reverse else v, w, dt)
