"""Method A with acados SQP/HPIPM. Same physical model and objective as SLSQP."""
import fcntl
import hashlib
import json
import math
import time

import numpy as np

from .acados_setup import acados, cache_dir
from .time_optimal import TimeOptimalMpc


class AcadosTimeOptimalMpc(TimeOptimalMpc):
    """Search integer horizons using a single resizable discrete OCP."""

    def __init__(self, params, dt=.1, max_steps=80, budget=.075,
                 pos_tol=.012, ang_tol=math.radians(2), wheel_tol=.12):
        super().__init__(params, dt, max_steps, budget, pos_tol, ang_tol,
                         wheel_tol, prepare_problems=False)
        self.candidate_timeout = min(.015, self.budget/5.)
        self.ocp = self.build_ocp()
        name = self.ocp.model.name
        folder = cache_dir(f'c_generated_code_{name}')
        json_path = cache_dir() / f'{name}.json'
        library = folder / f'libacados_ocp_solver_{name}.so'
        with (cache_dir() / f'{name}.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            fresh = not (library.is_file() and json_path.is_file())
            self.solver = acados().AcadosOcpSolver(
                self.ocp, json_file=str(json_path), generate=fresh,
                build=fresh, verbose=False)
        self.current_n = min(20, self.max_steps)
        self.attempt_statuses = []

    def build_ocp(self):
        import casadi as ca
        at = acados()
        # Five physical states + two normalized previous wheel commands.
        x = ca.SX.sym('x', 7)
        u = ca.SX.sym('u', 2)
        p = ca.SX.sym('p', 4)
        wh = x[3:5]/self.wmax
        vw = ca.DM(self.B) @ (u+ca.DM(self.avg)*(wh-u))
        h = self.dt*vw[1]
        nxt = ca.vertcat(x[0]+self.dt*vw[0]*ca.cos(x[2]+h/2),
                         x[1]+self.dt*vw[0]*ca.sin(x[2]+h/2), x[2]+h,
                         self.wmax*(u+ca.DM(self.decay)*(wh-u)),u)
        e = x[:3]-p[:3]
        g = ca.vertcat(ca.sumsqr(e[:2]/self.pos_tol),
                       e[2]/self.ang_tol, x[3:5]/self.wheel_tol,
                       x[5:7]*self.wmax/self.wheel_tol)
        flag = p[3]
        factor = .1+.9*flag
        model = at.AcadosModel()
        key = json.dumps(dict(version='acados-method-a-v1',
                              model=self.params.as_dict(), dt=self.dt,
                              n=self.max_steps, budget=self.candidate_timeout,
                              terminal=[self.pos_tol,self.ang_tol,self.wheel_tol]),
                         sort_keys=True)
        model.name = 'pinky_time_' + hashlib.sha256(key.encode()).hexdigest()[:12]
        model.x, model.u, model.p = x,u,p
        model.disc_dyn_expr = nxt
        # NLS costs are scaled by dt at nonterminal nodes.
        model.cost_y_expr = factor*ca.vertcat(
            math.sqrt(.02/self.dt)*u,
            math.sqrt(.08/self.dt)*(u-x[5:7]))
        model.cost_y_expr_e = ca.vertcat(
            .2*flag*g, math.sqrt(200)*(1-flag)*e[:2],
            math.sqrt(8)*(1-flag)*ca.sin(e[2]/2))
        model.con_h_expr = model.con_h_expr_0 = ca.vertcat(
            ca.DM(self.B)@u, u-x[5:7])
        model.con_h_expr_e = flag*g
        ocp = at.AcadosOcp()
        ocp.model = model
        ocp.parameter_values = np.array([0.,0.,0.,1.])
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = np.eye(4)
        ocp.cost.W_e = np.eye(9)
        ocp.cost.yref = np.zeros(4)
        ocp.cost.yref_e = np.zeros(9)
        c = ocp.constraints
        c.x0 = np.zeros(7)
        c.idxbu = np.arange(2)
        c.lbu,c.ubu = -np.ones(2),np.ones(2)
        hi = np.array([self.v_limit,self.w_limit,self.slew,self.slew])
        c.lh,c.uh = -hi,hi
        c.lh_0,c.uh_0 = -hi,hi
        c.lh_e,c.uh_e = -np.ones(6),np.ones(6)
        o = ocp.solver_options
        o.N_horizon = min(20,self.max_steps)
        o.tf = o.N_horizon*self.dt
        o.integrator_type = 'DISCRETE'
        o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        o.qp_solver_cond_N = 2
        o.hessian_approx = 'GAUSS_NEWTON'
        o.nlp_solver_type = 'SQP'
        o.nlp_solver_max_iter = 35
        o.globalization = 'MERIT_BACKTRACKING'
        o.levenberg_marquardt = 1e-5
        o.tol = 1e-5
        o.timeout_max_time = self.candidate_timeout
        o.timeout_heuristic = 'MAX_CALL'
        ocp.code_export_directory = str(cache_dir(f'c_generated_code_{model.name}'))
        return ocp

    def replay(self,x,us):
        xs = [np.asarray(x,float).copy()]
        for u in us:
            xs.append(self.step(xs[-1],u))
        return np.asarray(xs)

    def validate_candidate(self,x,goal,prev,us,vb,wb,forward,terminal):
        """Independently replay commands and check all applicable hard bounds."""
        if not np.all(np.isfinite(us)):
            return None, float('inf')
        z = np.asarray(us,float)/self.wmax
        if z.ndim != 2 or z.shape[1] != 2 or not len(z):
            return None, float('inf')
        vw = z@self.B.T
        du = np.diff(np.vstack([prev/self.wmax,z]),axis=0)
        lo = np.array([0. if forward else -vb,-wb])
        hi = np.array([vb,wb])
        violation = max(0.,float(np.max(np.abs(z)-1)),
                        float(np.max(lo-vw)),float(np.max(vw-hi)),
                        float(np.max(np.abs(du)-self.slew)))
        xs = self.replay(x,us)
        if not np.all(np.isfinite(xs)):
            return None, float('inf')
        if terminal:
            e = xs[-1,:3]-goal
            g = np.r_[np.sum((e[:2]/self.pos_tol)**2),
                      e[2]/self.ang_tol,xs[-1,3:5]/self.wheel_tol,
                      us[-1]/self.wheel_tol]
            violation = max(violation,float(np.max(np.abs(g)-1)))
        return xs,violation

    def attempt(self,n,x,goal,prev,vb,wb,forward,seed,deadline,terminal=True):
        started=time.perf_counter()
        if started>=deadline:
            return None
        s=self.solver
        if n!=self.current_n:
            s.set_new_time_steps(np.full(n,self.dt,dtype=np.float64))
            self.current_n=n
        s.reset()
        p=np.array([*goal,float(terminal)])
        hi=np.array([vb,wb,self.slew,self.slew])
        lo=-hi.copy()
        if forward:
            lo[0]=0.
        seed=np.asarray(seed,float)
        xs=self.replay(x,seed)
        old=prev
        for k in range(n):
            s.set(k,'p',p)
            s.set(k,'x',np.r_[xs[k],old/self.wmax])
            s.set(k,'u',seed[k]/self.wmax)
            s.constraints_set(k,'lh',lo)
            s.constraints_set(k,'uh',hi)
            old=seed[k]
        s.set(n,'p',p)
        s.set(n,'x',np.r_[xs[-1],old/self.wmax])
        initial=np.r_[x,prev/self.wmax]
        s.constraints_set(0,'lbx',initial)
        s.constraints_set(0,'ubx',initial)
        if time.perf_counter()>=deadline:
            return None
        status=int(s.solve())
        residuals=np.asarray(s.get_residuals(),float).ravel()
        iterations=int(s.get_stats('sqp_iter'))
        us=np.array([s.get(k,'u') for k in range(n)])*self.wmax
        predicted,violation=self.validate_candidate(
            x,goal,prev,us,vb,wb,forward,terminal)
        self.attempt_statuses.append(dict(n=int(n),status=status,
            terminal=bool(terminal),ms=round((time.perf_counter()-started)*1e3,3),
            violation=float(violation) if math.isfinite(violation) else None))
        # Nonconverged candidates can be feasible, never claim optimality.
        allowed=(0,2,7) if terminal else (0,)
        if (status not in allowed or predicted is None or violation>1e-5
                or len(residuals)<4 or not np.all(np.isfinite(residuals))
                or residuals[1]>1e-3 or residuals[2]>1e-3):
            return None
        message='acados converged' if status==0 else 'acados feasible iterate, not converged'
        return dict(u=tuple(us[0]),us=us,xs=predicted,ok=True,status=status,
                    iterations=iterations,message=message,residuals=residuals.tolist(),
                    replay_violation=float(violation),horizon_steps=n,
                    planned_time=n*self.dt,terminal_feasible=terminal,
                    cost=float(s.get_cost()))

    def solve(self,*args,**kwargs):
        self.attempt_statuses=[]
        result=super().solve(*args,**kwargs)
        result['backend']='discrete_time_optimal_acados'
        result['attempt_statuses']=self.attempt_statuses.copy()
        return result
