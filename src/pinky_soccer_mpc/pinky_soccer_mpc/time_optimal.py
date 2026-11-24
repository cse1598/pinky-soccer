"""Fixed-step, variable-horizon minimum-time pose NMPC (method A)."""
import math
import time
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint, minimize
from .pose_mpc import PoseMpc, wrap


class BudgetExpired(Exception):
    pass


class TimeOptimalMpc(PoseMpc):
    """Search terminal-feasible integer horizons, not reference tracking.

    A failed local NLP is not a proof of infeasibility. The budget is soft:
    SLSQP checks it between iterations, not during a native solver call.
    """
    def __init__(self, params, dt=.1, max_steps=80, budget=.075,
                 pos_tol=.012, ang_tol=math.radians(2), wheel_tol=.12):
        super().__init__(params, horizon=dt*20, steps=20)
        self.max_steps = int(max_steps)
        self.budget = float(budget)
        if self.max_steps < 2 or self.budget <= 0:
            raise ValueError('Invalid minimum-time search configuration')
        self.pos_tol, self.ang_tol, self.wheel_tol = pos_tol, ang_tol, wheel_tol
        self.cache = {}
        self.reset()
        # Compile all candidate sizes before control begins, not in the worker.
        for n in range(2, self.max_steps+1):
            self.problem(n)

    def reset(self):
        super().reset()
        self.plan = None
        self.plan_goal = None

    def problem(self, n):
        if n in self.cache:
            return self.cache[n]
        import casadi as ca
        z = ca.SX.sym('u', 2*n)
        x0 = ca.SX.sym('x', 5)
        goal = ca.SX.sym('goal', 3)
        prev = ca.SX.sym('prev', 2)
        x, old = x0, prev
        states = [x]
        effort = 0.
        for k in range(n):
            u = z[2*k:2*k+2]
            wh = x[3:5]/self.wmax
            vw = ca.DM(self.B) @ (u + ca.DM(self.avg)*(wh-u))
            h = self.dt*vw[1]
            x = ca.vertcat(x[0]+self.dt*vw[0]*ca.cos(x[2]+h/2),
                           x[1]+self.dt*vw[0]*ca.sin(x[2]+h/2), x[2]+h,
                           self.wmax*(u+ca.DM(self.decay)*(wh-u)))
            states.append(x)
            effort += .01*ca.sumsqr(u) + .04*ca.sumsqr(u-old)
            old = u
        e = x[:3]-goal
        # Both terminal wheel speed and last command must be near zero.
        g = ca.vertcat(ca.sumsqr(e[:2]/self.pos_tol), e[2]/self.ang_tol,
                       x[3:5]/self.wheel_tol, old*self.wmax/self.wheel_tol)
        # Tie-breaker ONLY within a fixed horizon; N takes precedence.
        cost = effort + .02*ca.sumsqr(g)
        # Explicit fallback if the bounded search finds no feasible arrival.
        progress = 100*ca.sumsqr(e[:2])+2*(1-ca.cos(e[2]))+.01*effort
        f = ca.Function('terminal_nlp', [z,x0,goal,prev],
                        [cost,ca.gradient(cost,z),g,ca.jacobian(g,z),
                         ca.horzcat(*states),progress,ca.gradient(progress,z)])
        d = np.eye(2*n)
        d[2:,:-2] -= np.eye(2*n-2)
        a = np.vstack([np.kron(np.eye(n),self.B),d])
        self.cache[n] = (f,a)
        return f,a

    def seed(self, n, x, goal, prev, vb, wb, forward, direction):
        # Old geometric rollout is ONLY an initial guess, never a cost reference.
        original = self.steps
        try:
            self.steps = n
            _, u = self.reference(x,goal,prev,vb,wb,direction)
        finally:
            self.steps = original
        return u

    def resize(self, us, n, prev, vb, wb, forward):
        if len(us) == 0:
            us = np.zeros((1,2))
        i = np.linspace(0,len(us)-1,n)
        out = np.column_stack([np.interp(i,np.arange(len(us)),us[:,j]) for j in range(2)])
        for k in range(n):
            out[k] = self.project(out[k],prev,vb,wb,1 if forward else 0)
            prev = out[k]
        return out

    def attempt(self,n,x,goal,prev,vb,wb,forward,seed,deadline,terminal=True):
        f,a = self.problem(n)
        args = (x,goal,prev/self.wmax)
        lo = np.r_[np.tile([0 if forward else -vb,-wb],n),np.full(2*n,-self.slew)]
        hi = np.r_[np.tile([vb,wb],n),np.full(2*n,self.slew)]
        lo[2*n:2*n+2] += prev/self.wmax
        hi[2*n:2*n+2] += prev/self.wmax
        gl = np.array([-np.inf,-1,-1,-1,-1,-1])
        gh = np.ones(6)
        cache_z, cache_out = None, None
        best = None
        z0=(seed/self.wmax).ravel()

        def eval_at(z, accepted=False):
            nonlocal cache_z,cache_out,best
            if cache_z is None or not np.array_equal(z,cache_z):
                cache_z = z.copy()
                cache_out = f(z,*args)
                g = np.asarray(cache_out[2]).ravel()
                az = a@z
                violation = max(0.,np.max(lo-az),np.max(az-hi),np.max(np.abs(z))-1.)
                if terminal:
                    violation=max(violation,np.max(g-gh),np.max(gl-g))
                if (np.all(np.isfinite(z)) and np.all(np.isfinite(g)) and
                        violation <= 1e-6 and (accepted or not np.array_equal(z,z0))):
                    score=float(cache_out[0 if terminal else 5])
                    if math.isfinite(score) and (best is None or score<best[0]):
                        best=(score,z.copy(),np.asarray(cache_out[4]).T,violation)
            return cache_out

        def objective(z):
            out=eval_at(z)
            return float(out[0 if terminal else 5]),np.asarray(out[1 if terminal else 6]).ravel()

        def callback(z):
            eval_at(z)
            if time.perf_counter()>=deadline:
                raise BudgetExpired()

        constraints=[LinearConstraint(a,lo,hi)]
        if terminal:
            constraints.append(NonlinearConstraint(
                lambda z:np.asarray(eval_at(z)[2]).ravel(),gl,gh,
                jac=lambda z:np.asarray(eval_at(z)[3])))
        status=-2
        iterations=0
        message='search budget reached'
        try:
            res=minimize(objective,z0,jac=True,method='SLSQP',bounds=Bounds(-1.,1.),
                         constraints=constraints,callback=callback,
                         options={'maxiter':30,'ftol':1e-6})
            cache_z=None
            eval_at(res.x,accepted=bool(res.success))
            status=int(res.status)
            iterations=int(res.nit)
            message=str(res.message)
            if not terminal and not res.success:
                best=None
        except BudgetExpired:
            if not terminal:
                best=None
        if best is None:
            return None
        score,z,xs,violation=best
        us=z.reshape(-1,2)*self.wmax
        return dict(u=tuple(us[0]),us=us,xs=xs,ok=True,status=status,
                    iterations=iterations,message=message,residuals=[float(violation)],
                    horizon_steps=n,planned_time=n*self.dt,terminal_feasible=terminal,
                    cost=score)

    def solve(self,x0,goal,v_cap=None,w_cap=None,forward_only=False):
        start=time.perf_counter()
        deadline=start+self.budget
        x0=np.asarray(x0,float)
        goal=np.asarray(goal,float).copy()
        x=x0[:5]
        goal[2]=x[2]+wrap(goal[2]-x[2])
        vb=min(self.v_limit,v_cap) if v_cap and v_cap>0 else self.v_limit
        wb=min(self.w_limit,w_cap) if w_cap and w_cap>0 else self.w_limit
        prev=np.asarray(x0[-2:])
        d=np.linalg.norm(goal[:2]-x[:2])
        angle=abs(goal[2]-x[2])
        bearing=np.arctan2(goal[1]-x[1],goal[0]-x[0])
        front=abs(wrap(bearing-x[2]))+abs(wrap(goal[2]-bearing))
        back=abs(wrap(bearing+np.pi-x[2]))+abs(wrap(goal[2]-bearing-np.pi))
        direction=1 if forward_only or front<=back else -1
        if d<.02:
            front=back=angle
        speed=max(vb,abs((self.rl*x[3]+self.rr*x[4])/2))
        spin=max(wb,abs((self.rr*x[4]-self.rl*x[3])/self.b))
        # Optimistic lower estimate; actual minimum may be larger.
        lower=max(2,math.ceil(max(0.,(d-self.pos_tol)/speed,
                                  (angle-self.ang_tol)/spin)/self.dt))
        same=(self.plan_goal is not None and
              np.linalg.norm(goal[:2]-self.plan_goal[:2])<1e-6 and
              abs(wrap(goal[2]-self.plan_goal[2]))<1e-6)
        warm=self.plan[1:] if same and self.plan is not None and len(self.plan)>1 else None
        guess=max(lower,min(self.max_steps,math.ceil(
            (d/vb+min(front,back if not forward_only else front)/wb+.45)/self.dt)))
        n=min(self.max_steps,max(lower,len(warm) if warm is not None else guess))
        best=None
        attempts=0
        tried=set()
        failed_below=lower-1
        # Find a feasible horizon, then shorten it. Failed local solves are not
        # infeasibility certificates; this is a bounded heuristic search.
        while lower<=self.max_steps and attempts<4 and n not in tried and (
                attempts==0 or time.perf_counter()<deadline):
            tried.add(n)
            attempts+=1
            source=best['us'] if best is not None else warm
            seed=(self.resize(source,n,prev,vb,wb,forward_only) if source is not None
                  else self.seed(n,x,goal,prev,vb,wb,forward_only,direction))
            cand=self.attempt(n,x,goal,prev,vb,wb,forward_only,seed,deadline)
            if cand is not None:
                best=cand
                if n<=lower:
                    break
                n=max(lower,(failed_below+n)//2)
            elif best is not None:
                failed_below=max(failed_below,n)
                if best['horizon_steps']<=failed_below+1:
                    break
                n=(failed_below+best['horizon_steps'])//2
            else:
                if n==self.max_steps:
                    break
                n=min(self.max_steps,n+max(3,n//4))
        if best is None:
            # Distant/unresolved targets: optimized progress, explicitly not a
            # terminal-feasible plan. No raw seed or P command is transmitted.
            n=min(self.max_steps,20)
            seed=self.seed(n,x,goal,prev,vb,wb,forward_only,direction)
            best=self.attempt(n,x,goal,prev,vb,wb,forward_only,seed,
                              max(deadline,time.perf_counter()+.018),terminal=False)
            if best is not None:
                best['planned_time']=None
        if best is None:
            self.plan=None
            best=dict(u=(0.,0.),us=np.zeros((1,2)),xs=np.array([x]),ok=False,status=-1,
                      iterations=0,message='no feasible horizon or converged progress solution',
                      residuals=[],horizon_steps=None,planned_time=None,terminal_feasible=False)
        else:
            self.plan=best['us'].copy() if best['terminal_feasible'] else None
            self.plan_goal=goal.copy()
        best.update(ms=(time.perf_counter()-start)*1000,v_bound=vb,w_bound=wb,
                    direction=direction,attempts=attempts,lower_steps=lower,
                    backend='discrete_time_optimal_slsqp')
        return best
