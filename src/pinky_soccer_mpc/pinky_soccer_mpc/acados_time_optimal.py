"""Method A: elastic terminal SQP, with strict independent replay acceptance."""
import fcntl
import hashlib
import json
import math
import time

import numpy as np

from pinky_soccer_dynamics import profile_model as profile

from .acados_setup import acados, cache_dir
from .ramp_predictor import RampPredictor
from . import damping
from .time_optimal import TimeOptimalMpc


class AcadosTimeOptimalMpc(TimeOptimalMpc):
    """Search integer horizons using a single resizable discrete OCP."""

    state_size = 7

    def __init__(self, params, dt=.1, max_steps=80, budget=.075,
                 pos_tol=.012, ang_tol=math.radians(2), wheel_tol=.12,
                 damping_scale=1.):
        super().__init__(params, dt, max_steps, budget, pos_tol, ang_tol,
                         wheel_tol, prepare_problems=False)
        self.damping_scale=float(damping_scale)
        if not math.isfinite(self.damping_scale) or self.damping_scale<0:
            raise ValueError('damping_scale must be finite and nonnegative')
        self.candidate_timeout = min(.022, self.budget/3.5)
        self.retry_seed = None
        self.predictor=RampPredictor(params,self.dt)
        self.ocp = self.build_ocp()
        name = self.ocp.model.name
        folder = cache_dir(f'c_generated_code_{name}')
        json_path = folder / f'{name}.json'
        library = folder / f'libacados_ocp_solver_{name}.so'
        with (cache_dir() / f'{name}.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            fresh = not (library.is_file() and json_path.is_file())
            self.solver = acados().AcadosOcpSolver(
                self.ocp, generate=fresh, build=fresh, verbose=False)
        self.current_n = min(20, self.max_steps)
        self.attempt_statuses = []

    def build_ocp(self):
        import casadi as ca
        at = acados()
        # Pose, measured wheels, motor profile, previous command; normalized
        # wheel states inside SQP. Exact same 5 ms ramp map as profile_model.
        x = ca.SX.sym('x', 9)
        u = ca.SX.sym('u', 2)
        p = ca.SX.sym('p', 7)  # goal, terminal flag, v/w caps, sqrt(damping scale)
        px,pu,pnext=profile.symbolic_step(self.params,self.dt)
        f=ca.Function('profile_transition',[px,pu],[pnext])
        physical=f(ca.vertcat(x[:3],x[3:9]*self.wmax),u*self.wmax)
        nxt=ca.vertcat(physical[:3],physical[3:9]/self.wmax)
        e = x[:3]-p[:3]
        # Linear, well-scaled terminal constraints replace the squared radius
        # constraint in the SQP subproblem. Four two-sided projections form
        # an inscribed octagon (conservative by <=7.6% in radial extent).
        # The final replay still uses the original exact circular tolerance.
        angles = np.arange(4)*np.pi/4
        facets = ca.DM(np.column_stack((np.cos(angles),np.sin(angles))))
        g = ca.vertcat(facets@(e[:2]/self.pos_tol)/math.cos(math.pi/8),
                       e[2]/self.ang_tol,
                       x[3:5]*self.wmax/self.wheel_tol,
                       x[5:9]*self.wmax/self.wheel_tol)
        center = ca.vertcat(e[:2]/self.pos_tol, e[2]/self.ang_tol,
                            x[3:9]*self.wmax/self.wheel_tol)
        flag = p[3]
        factor = .1+.9*flag
        model = at.AcadosModel()
        key = json.dumps(dict(version='acados-method-a-v4-damping',
                              model=self.params.as_dict(), dt=self.dt,
                              n=self.max_steps, budget=self.candidate_timeout,
                              terminal=[self.pos_tol,self.ang_tol,self.wheel_tol],
                              damping=[damping.RADIUS_M,damping.HEADING_RADIUS_RAD,
                                       damping.Q_V,damping.Q_W]),
                         sort_keys=True)
        model.name = 'pinky_time_' + hashlib.sha256(key.encode()).hexdigest()[:12]
        model.x, model.u, model.p = x,u,p
        model.disc_dyn_expr = nxt
        # NLS costs are scaled by dt at nonterminal nodes.
        # N remains the primary objective. Penalize physical speed only
        # within a terminal candidate, not the progress-only fallback.
        # Heading gate avoids braking a necessary large in-place rotation.
        vw = ca.DM(self.B)@x[3:5]
        damp = damping.symbolic_residual(e,vw,p[4],p[5],flag*p[6])
        model.cost_y_expr = ca.vertcat(
            factor*math.sqrt(.02/self.dt)*u,
            factor*math.sqrt(.08/self.dt)*(u-x[7:9]),damp)
        model.cost_y_expr_e = ca.vertcat(
            .5*flag*center, math.sqrt(200)*(1-flag)*e[:2],
            math.sqrt(8)*(1-flag)*ca.sin(e[2]/2))
        model.con_h_expr = model.con_h_expr_0 = ca.vertcat(
            ca.DM(self.B)@u, u-x[7:9])
        model.con_h_expr_e = flag*g
        ocp = at.AcadosOcp()
        ocp.model = model
        ocp.parameter_values = np.array([0.,0.,0.,1.,self.v_limit,self.w_limit,
                                         math.sqrt(self.damping_scale)])
        ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = np.eye(6)
        ocp.cost.W_e = np.eye(12)
        ocp.cost.yref = np.zeros(6)
        ocp.cost.yref_e = np.zeros(12)
        c = ocp.constraints
        c.x0 = np.zeros(9)
        c.idxbu = np.arange(2)
        c.lbu,c.ubu = -np.ones(2),np.ones(2)
        hi = np.array([self.v_limit,self.w_limit,self.slew,self.slew])
        c.lh,c.uh = -hi,hi
        c.lh_0,c.uh_0 = -hi,hi
        nh = 11
        c.lh_e,c.uh_e = -np.ones(nh),np.ones(nh)
        c.idxsh_e = np.arange(nh)
        c.lsh_e = np.zeros(nh)
        c.ush_e = np.zeros(nh)
        # Elastic terminal only. Wheel command, body and slew limits stay hard.
        ocp.cost.zl_e = ocp.cost.zu_e = 30.*np.ones(nh)
        ocp.cost.Zl_e = ocp.cost.Zu_e = 60.*np.ones(nh)
        o = ocp.solver_options
        o.N_horizon = min(20,self.max_steps)
        o.tf = o.N_horizon*self.dt
        o.integrator_type = 'DISCRETE'
        o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        o.qp_solver_cond_N = 2
        o.hessian_approx = 'GAUSS_NEWTON'
        o.nlp_solver_type = 'SQP'
        o.nlp_solver_max_iter = 100
        o.qp_solver_iter_max = 100
        o.hpipm_mode = 'ROBUST'
        o.globalization = 'MERIT_BACKTRACKING'
        o.levenberg_marquardt = 1e-3
        o.tol = 1e-4
        o.timeout_max_time = self.candidate_timeout
        o.timeout_heuristic = 'MAX_CALL'
        ocp.code_gen_options.code_export_directory = str(cache_dir(f'c_generated_code_{model.name}'))
        ocp.code_gen_options.json_file = f'{model.name}.json'
        return ocp

    def step(self,x,u):
        return self.predictor.step(x,u)

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
                      e[2]/self.ang_tol,xs[-1,3:7]/self.wheel_tol,
                      us[-1]/self.wheel_tol]
            violation = max(violation,float(np.max(np.abs(g)-1)))
        return xs,violation

    def pack_state(self,x,prev):
        """Convert physical replay state to scaled solver coordinates."""
        return np.r_[x[:3],x[3:7]/self.wmax,prev/self.wmax]

    def terminal_errors(self,xs,us,goal):
        e=xs[-1,:3]-goal
        return dict(position_m=float(np.linalg.norm(e[:2])),
                    heading_rad=float(abs(e[2])),
                    wheel_rad_s=float(np.max(np.abs(xs[-1,3:5]))),
                    profile_rad_s=float(np.max(np.abs(xs[-1,5:7]))),
                    command_rad_s=float(np.max(np.abs(us[-1]))))

    def terminal_merit(self,xs,us,goal):
        e=self.terminal_errors(xs,us,goal)
        return ((e['position_m']/self.pos_tol)**2+
                (e['heading_rad']/self.ang_tol)**2+
                (e['wheel_rad_s']/self.wheel_tol)**2+
                (e['profile_rad_s']/self.wheel_tol)**2+
                (e['command_rad_s']/self.wheel_tol)**2)

    def progress_merit(self,xs,us,prev,goal):
        e=xs[-1,:3]-goal
        z=us/self.wmax
        du=np.diff(np.vstack([prev/self.wmax,z]),axis=0)
        return float(100*np.dot(e[:2],e[:2])+2*(1-np.cos(e[2]))+
                     .01*(.01*np.sum(z*z)+.04*np.sum(du*du)))

    def attempt(self,n,x,goal,prev,vb,wb,forward,seed,deadline,terminal=True):
        started=time.perf_counter()
        if started>=deadline:
            return None
        s=self.solver
        if n!=self.current_n:
            s.set_new_time_steps(np.full(n,self.dt,dtype=np.float64))
            self.current_n=n
        s.reset()
        p=np.array([*goal,float(terminal),vb,wb,math.sqrt(self.damping_scale)])
        hi=np.array([vb,wb,self.slew,self.slew])
        lo=-hi.copy()
        if forward:
            lo[0]=0.
        seed=np.asarray(seed,float)
        xs=self.replay(x,seed)
        # Reuse a rejected elastic solution only as a numerical initialization.
        # A shorter candidate may have useful geometry even if it cannot stop.
        if terminal and self.retry_seed is not None:
            retry=self.resize(self.retry_seed,n,prev,vb,wb,forward)
            rx=self.replay(x,retry)
            if self.terminal_merit(rx,retry,goal)<self.terminal_merit(xs,seed,goal):
                seed,xs=retry,rx
        old=prev
        for k in range(n):
            s.set(k,'p',p)
            s.set(k,'x',self.pack_state(xs[k],old))
            s.set(k,'u',seed[k]/self.wmax)
            s.constraints_set(k,'lh',lo)
            s.constraints_set(k,'uh',hi)
            old=seed[k]
        s.set(n,'p',p)
        s.set(n,'x',self.pack_state(xs[-1],old))
        initial=self.pack_state(x,prev)
        # Initialize terminal slack variables consistently with the seed.
        if terminal:
            angles=np.arange(4)*np.pi/4
            facets=np.column_stack((np.cos(angles),np.sin(angles)))
            e=xs[-1,:3]-goal
            g=np.r_[facets@(e[:2]/self.pos_tol)/math.cos(math.pi/8),
                    e[2]/self.ang_tol,xs[-1,3:7]/self.wheel_tol,
                    old/self.wheel_tol]
        else:
            g=np.zeros(11)
        s.set(n,'sl',np.maximum(0.,-1-g))
        s.set(n,'su',np.maximum(0.,g-1))
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
        def finite_list(values):
            return [float(v) if math.isfinite(v) else None for v in values]
        errors=self.terminal_errors(predicted,us,goal) if predicted is not None else None
        slack=np.r_[s.get(n,'sl'),s.get(n,'su')]
        cost=float(s.get_cost())
        residual_ok=(len(residuals)>=4 and np.all(np.isfinite(residuals))
                     and residuals[1]<=1e-3 and residuals[2]<=1e-3)
        safe_status=status in (0,2,7) and math.isfinite(cost)
        if terminal and predicted is not None and safe_status:
            _,path_violation=self.validate_candidate(
                x,goal,prev,us,vb,wb,forward,False)
            if path_violation<=1e-5:
                self.retry_seed=us.copy()
        accepted=bool(safe_status and predicted is not None and violation<=1e-5
                      and residual_ok)
        if accepted and not terminal:
            # For progress-only fallback, require improvement over a feasible
            # braking rollout even if SQP reaches its iteration/time limit.
            brake=[]
            old=prev
            for _ in range(n):
                old=self.project(np.zeros(2),old,vb,wb,1 if forward else 0)
                brake.append(old)
            brake=np.asarray(brake)
            bxs,bviolation=self.validate_candidate(x,goal,prev,brake,vb,wb,forward,False)
            accepted=bool(bxs is not None and bviolation<=1e-5 and
                self.progress_merit(predicted,us,prev,goal) <
                self.progress_merit(bxs,brake,prev,goal)-1e-8)
        damp_cost=(damping.trajectory_cost(predicted,goal,self.B,self.wmax,
                        self.dt,vb,wb,self.damping_scale) if terminal and predicted is not None else 0.)
        self.attempt_statuses.append(dict(n=int(n),status=status,
            terminal=bool(terminal),ms=round((time.perf_counter()-started)*1e3,3),
            iterations=iterations,residuals=finite_list(residuals),
            slack_max=float(np.max(slack)) if np.all(np.isfinite(slack)) else None,
            terminal_errors=errors,accepted=accepted,damping_cost=damp_cost,
            violation=float(violation) if math.isfinite(violation) else None))
        if not accepted:
            return None
        # Nonconverged results are independently feasible, not certified optimal.
        message='acados converged' if status==0 else 'acados feasible iterate, not converged'
        return dict(u=tuple(us[0]),us=us,xs=predicted,ok=True,status=status,
                    iterations=iterations,message=message,residuals=residuals.tolist(),
                    replay_violation=float(violation),horizon_steps=n,
                    planned_time=n*self.dt,terminal_feasible=terminal,
                    cost=cost,damping_cost=damp_cost)

    def solve(self,*args,**kwargs):
        self.attempt_statuses=[]
        self.retry_seed=None
        result=super().solve(*args,**kwargs)
        result['backend']='discrete_time_optimal_acados'
        result['attempt_statuses']=self.attempt_statuses.copy()
        result['damping_scale']=self.damping_scale
        return result
