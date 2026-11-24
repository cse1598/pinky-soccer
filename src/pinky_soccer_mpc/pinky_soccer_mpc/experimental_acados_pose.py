"""Point-to-pose NMPC: motor profile, wheel lag, and command continuity.

State [x,y,theta,wl,wr,ref_l,ref_r,last_cmd_l,last_cmd_r].
Input: target wheel angular velocities. Seven physical states and two command
memory states. A direction is selected once per substantially changed goal.
"""
import hashlib
import json
import time

import numpy as np
from pinky_soccer_dynamics import profile_model as dyn

from .acados_setup import acados, cache_dir

DEFAULT_WEIGHTS = {
    'pos': 12., 'time': 18., 'eps': .02, 'head': 10., 'travel_head': 5.,
    'head_radius': .15, 'acc': .35, 'delta': 1.2, 'delta_turn': 2.5,
    'pos_e': 400., 'head_e': 45., 'speed_e': 8., 'time_e': 35.,
    'time_radius': .10, 'slack': 1e3,
}


def wrap(a):
    return np.arctan2(np.sin(a),np.cos(a))


def choose_direction(x, goal, forward_only=False):
    """Compare signed front/back bearings including final turn; bias to forward."""
    if forward_only:
        return 1
    dx, dy = np.asarray(goal[:2])-np.asarray(x[:2])
    if np.hypot(dx,dy) < .05:
        return 1
    bearing = np.arctan2(dy,dx)
    f = abs(wrap(bearing-x[2])) + abs(wrap(goal[2]-bearing))
    back = bearing+np.pi
    r = abs(wrap(back-x[2])) + abs(wrap(goal[2]-back))
    return -1 if r+.25 < f else 1


class PoseMpc:
    """Generate one cached discrete acados OCP per model and tuning set."""

    def __init__(self, params, horizon=2., steps=40, weights=None, name='pinky_pose'):
        self.params = params
        self.horizon, self.steps = float(horizon),int(steps)
        self.dt = self.horizon/self.steps
        self.w = dict(DEFAULT_WEIGHTS, **(weights or {}))
        self.wmax, self.amax = params.wheel_cmd_max(),params.wheel_accel_cmd_max()
        self.rl,self.rr,self.b = (params.m(k) for k in ('left_radius','right_radius','track'))
        self.v_limit = .5*(self.rl+self.rr)*self.wmax
        self.w_limit = (self.rl+self.rr)*self.wmax/self.b
        self.ocp = self._build(name)
        lib = cache_dir(f'c_generated_code_{self.ocp.model.name}') / f'libacados_ocp_solver_{self.ocp.model.name}.so'
        fresh = not lib.exists()
        self.solver = acados().AcadosOcpSolver(self.ocp, json_file=f'{self.ocp.model.name}.json',
                                                build=fresh,generate=fresh,verbose=False)
        self.reset()

    def _build(self, name):
        import casadi as ca
        at = acados()
        x,u,nxt = dyn.symbolic_step(self.params,self.dt)
        p = ca.SX.sym('p',6)  # goal, locked direction, v/w caps
        w = self.w
        key = json.dumps({'version':dyn.MODEL_VERSION,'model':self.params.as_dict(),
                          'weights':w,'h':[self.horizon,self.steps]},sort_keys=True)
        model = at.AcadosModel()
        model.name = f'{name}_{hashlib.sha1(key.encode()).hexdigest()[:10]}'
        model.x,model.u,model.p,model.disc_dyn_expr = x,u,p,nxt
        dx,dy = p[0]-x[0],p[1]-x[1]
        d2 = dx*dx+dy*dy
        dist = ca.sqrt(d2+1e-6)
        near = w['head_radius']**2/(d2+w['head_radius']**2)
        # Negate both bearing components for reverse, rather than folding only cosine.
        bx,by = p[3]*dx/dist,p[3]*dy/dist
        travel_cos = bx*ca.cos(x[2])+by*ca.sin(x[2])
        final_cos = bx*ca.cos(p[2])+by*ca.sin(p[2])
        head = ca.sin(.5*(x[2]-p[2]))
        s = d2/(d2+w['time_radius']**2)
        e1 = ca.sqrt(2*(1-travel_cos)+1e-4)
        e2 = ca.sqrt(2*(1-final_cos)+1e-4)
        tgo = dist/p[4]+(s*(e1+e2)+(1-s)*ca.sqrt(4*head**2+1e-4))/p[5]
        du = u-x[7:9]
        dw = (self.rr*du[1]-self.rl*du[0])/self.b
        acc = (nxt[3:5]-x[3:5])/self.dt
        model.cost_y_expr = ca.vertcat((d2+w['eps']**2)**.25,
            ca.sqrt(2*near)*head,ca.sqrt(s+1e-6)*e1,ca.sqrt(tgo+1e-6),
            acc/self.amax,du/(self.amax*self.dt),dw/self.w_limit)
        model.cost_y_expr_e = ca.vertcat(dx,dy,ca.sqrt(2.)*head,
            x[3:7]/self.wmax,ca.sqrt(tgo+1e-6))
        def twist(q):
            return ca.vertcat((self.rl*q[0]+self.rr*q[1])/2,
                              (self.rr*q[1]-self.rl*q[0])/self.b)
        model.con_h_expr = model.con_h_expr_0 = ca.vertcat(twist(u),twist(x[3:5]),du,acc)
        ocp = at.AcadosOcp()
        ocp.model = model
        ocp.parameter_values = np.array([0.,0.,0.,1.,self.v_limit,self.w_limit])
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = np.diag([w['pos'],w['head'],w['travel_head'],w['time'],
                              w['acc'],w['acc'],w['delta'],w['delta'],w['delta_turn']])
        ocp.cost.W_e = np.diag([w['pos_e'],w['pos_e'],w['head_e'],*([w['speed_e']]*4),w['time_e']])
        ocp.cost.yref,ocp.cost.yref_e = np.zeros(9),np.zeros(8)
        c = ocp.constraints
        c.lbu,c.ubu,c.idxbu = -self.wmax*np.ones(2),self.wmax*np.ones(2),np.arange(2)
        bounds = np.array([self.v_limit,self.w_limit,self.v_limit,self.w_limit,
                           self.amax*self.dt,self.amax*self.dt,self.amax,self.amax])
        c.lh,c.uh = -bounds,bounds
        c.lh_0,c.uh_0 = -bounds,bounds
        c.idxsh = c.idxsh_0 = np.arange(8)
        slack = w['slack']*np.ones(8)
        ocp.cost.zl = ocp.cost.zu = ocp.cost.Zl = ocp.cost.Zu = slack
        ocp.cost.zl_0 = ocp.cost.zu_0 = ocp.cost.Zl_0 = ocp.cost.Zu_0 = slack
        c.x0 = np.zeros(dyn.NX)
        o = ocp.solver_options
        o.N_horizon,o.tf = self.steps,self.horizon
        o.integrator_type = 'DISCRETE'
        o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        o.hessian_approx = 'GAUSS_NEWTON'
        o.nlp_solver_type = 'SQP'
        o.nlp_solver_max_iter = 20
        o.tol = 1e-3
        o.globalization = 'MERIT_BACKTRACKING'
        ocp.code_export_directory = str(cache_dir(f'c_generated_code_{model.name}'))
        return ocp

    def project(self,u,prev,vb,wb,direction=0):
        """Ordinary commands slew limited; hard current speed caps take precedence."""
        delta = self.amax*self.dt
        u = np.asarray(prev)+np.clip(np.asarray(u)-prev,-delta,delta)
        return dyn.limit_command(self.params,u,vb,wb,direction)

    def primitive_guess(self,x0,goal,direction,vb,wb):
        x = np.asarray(x0,float).copy()
        xs,us = [x.copy()],[]
        for _ in range(self.steps):
            dx,dy = np.asarray(goal[:2])-x[:2]
            dist = np.hypot(dx,dy)
            if dist > .045:
                err = wrap(np.arctan2(dy,dx)+(np.pi if direction<0 else 0)-x[2])
                v = direction*min(vb,2.*dist)*max(0.,np.cos(err))**2
            else:
                err,v = wrap(goal[2]-x[2]),0.
            w = np.clip(3.*err,-wb,wb)
            u = self.project([(v-self.b*w/2)/self.rl,(v+self.b*w/2)/self.rr],
                             x[7:9],vb,wb,direction)
            x = dyn.step(self.params,x,u,self.dt)
            xs.append(x.copy())
            us.append(u)
        return np.asarray(xs),np.asarray(us)

    def solve(self,x0,goal,v_cap=None,w_cap=None,forward_only=False):
        start = time.perf_counter()
        s,n = self.solver,self.steps
        x0,goal = np.asarray(x0,float),np.asarray(goal,float)
        if x0.shape != (dyn.NX,):
            raise ValueError('NMPC requires nine states including profile and previous command')
        vb = min(self.v_limit,v_cap) if v_cap and v_cap>0 else self.v_limit
        wb = min(self.w_limit,w_cap) if w_cap and w_cap>0 else self.w_limit
        changed = (self.anchor is None or forward_only != self.forward_only
                   or np.linalg.norm(goal[:2]-self.anchor[:2])>.15
                   or abs(wrap(goal[2]-self.anchor[2]))>.7)
        if changed:
            self.reset()
            self.anchor,self.forward_only = goal.copy(),forward_only
            self.direction = choose_direction(x0,goal,forward_only)
        p = np.array([*goal,self.direction,vb,wb])
        hi = np.array([vb,wb,vb,wb,self.amax*self.dt,self.amax*self.dt,self.amax,self.amax])
        lo = -hi.copy()
        if self.direction==1:
            lo[0]=lo[2]=0.
        else:
            hi[0]=hi[2]=0.
        for k in range(n+1):
            s.set(k,'p',p)
        for k in range(n):
            s.constraints_set(k,'lh',lo)
            s.constraints_set(k,'uh',hi)
        if self.us is None:
            xs,us = self.primitive_guess(x0,goal,self.direction,vb,wb)
        else:
            us = np.vstack([self.us[1:],self.us[-1:]])
            xs = [x0.copy()]
            for k in range(n):
                us[k] = self.project(us[k],xs[-1][7:9],vb,wb,self.direction)
                xs.append(dyn.step(self.params,xs[-1],us[k],self.dt))
        for k in range(n+1):
            s.set(k,'x',xs[k])
        for k in range(n):
            s.set(k,'u',us[k])
        raw = s.solve_for_x0(x0,fail_on_nonzero_status=False,print_stats_on_failure=False)
        status = int(s.status)
        xs = np.array([s.get(k,'x') for k in range(n+1)])
        us = np.array([s.get(k,'u') for k in range(n)])
        residuals = np.asarray(s.get_residuals(),float)
        finite = np.all(np.isfinite(xs)) and np.all(np.isfinite(us)) and np.all(np.isfinite(residuals))
        ok = bool(finite and (status==0 or (status==2 and residuals[1]<.02 and residuals[2]<.02)))
        cmd = self.project(raw,x0[7:9],vb,wb,self.direction) if ok else np.zeros(2)
        self.us = us if ok else None
        return {'u':tuple(cmd),'status':status,'ok':ok,'xs':xs,'us':us,
                'ms':(time.perf_counter()-start)*1000,'v_bound':vb,'w_bound':wb,
                'direction':self.direction,'residuals':residuals.tolist() if finite else []}

    def reset(self):
        self.us = None
        self.anchor = None
        self.direction = 1
        self.forward_only = False
