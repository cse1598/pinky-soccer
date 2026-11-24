"""Phase-1 smooth wheel-lag NMPC, direct shooting with CasADi AD and SLSQP.
Only optimized commands are transmitted. The geometric rollout is a reference/seed.
"""
import time
import numpy as np
from scipy.optimize import minimize, LinearConstraint, Bounds


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class PoseMpc:
    def __init__(self, params, horizon=2., steps=20, weights=None, name='pinky_pose'):
        import casadi as ca
        self.params = params
        self.steps, self.dt = int(steps), float(horizon)/int(steps)
        self.wmax, self.amax = params.wheel_cmd_max(), params.wheel_accel_cmd_max()
        self.rl,self.rr,self.b = (params.m(k) for k in ('left_radius','right_radius','track'))
        self.v_limit = .5*(self.rl+self.rr)*self.wmax
        self.w_limit = (self.rl+self.rr)*self.wmax/self.b
        self.B = np.array([[self.rl/2,self.rr/2],[-self.rl/self.b,self.rr/self.b]])*self.wmax
        self.slew = self.amax*self.dt/self.wmax
        self.decay = np.exp(-self.dt/np.array([params.m('tau_left'),params.m('tau_right')]))
        self.avg = np.array([params.m('tau_left'),params.m('tau_right')])*(1-self.decay)/self.dt
        n = self.steps
        z = ca.SX.sym('z',2*n); x0 = ca.SX.sym('x0',5)
        ref = ca.SX.sym('ref',3*(n+1)); prev = ca.SX.sym('prev',2)
        x = x0; states = [x]; cost = 0.; old = prev
        for k in range(n):
            u = z[2*k:2*k+2]
            wh = x[3:5]/self.wmax
            midwh = u+ca.DM(self.avg)*(wh-u)
            vw = ca.DM(self.B)@midwh
            dtheta = self.dt*vw[1]
            x = ca.vertcat(x[0]+self.dt*vw[0]*ca.cos(x[2]+dtheta/2),
                           x[1]+self.dt*vw[0]*ca.sin(x[2]+dtheta/2),x[2]+dtheta,
                           self.wmax*(u+ca.DM(self.decay)*(wh-u)))
            e = x[:3]-ref[3*(k+1):3*(k+2)]
            du = u-old
            cost += self.dt*(100*ca.sumsqr(e[:2])+2*e[2]**2+.12*ca.sumsqr(u)
                             +.65*ca.sumsqr(du)+.25*(du[1]-du[0])**2)
            states.append(x); old=u
        cost += 350*ca.sumsqr(e[:2])+12*e[2]**2+2*ca.sumsqr(x[3:5]/self.wmax)
        self.fun = ca.Function('pose_cost',[z,x0,ref,prev],[cost,ca.gradient(cost,z),ca.horzcat(*states)])
        self.Abody = np.kron(np.eye(n),self.B)
        D = np.eye(2*n)
        D[2:,:-2] -= np.eye(2*n-2)
        self.A = np.vstack([self.Abody,D])
        self.reset()

    def reset(self):
        self.us=None
        self.direction=1
        self.anchor=None

    def project(self,u,prev,vb,wb,direction=0):
        # Project previous and requested target onto body caps and wheel limits.
        def caps(z):
            v,w = self.B@z
            v = np.clip(v,0 if direction>0 else -vb,0 if direction<0 else vb)
            w = np.clip(w,-wb,wb)
            z = np.linalg.solve(self.B,[v,w])
            return z/max(1.,np.max(np.abs(z)))
        prev=caps(np.asarray(prev)/self.wmax)
        u=caps(np.asarray(u)/self.wmax)
        return self.wmax*(prev+(u-prev)*min(1.,self.slew/max(np.max(np.abs(u-prev)),1e-12)))

    def step(self,x,u):
        x=np.asarray(x,float); u=np.asarray(u,float)/self.wmax
        wh=x[3:5]/self.wmax
        v,w=self.B@(u+self.avg*(wh-u)); h=self.dt*w
        return np.r_[x[:2]+self.dt*v*np.array([np.cos(x[2]+h/2),np.sin(x[2]+h/2)]),
                     x[2]+h,self.wmax*(u+self.decay*(wh-u))]

    def reference(self,x,goal,prev,vb,wb,direction):
        xs=[x.copy()]; us=[]
        for k in range(self.steps):
            dx,dy=goal[:2]-x[:2]; d=np.hypot(dx,dy)
            if d>.018:
                err=wrap(np.arctan2(dy,dx)+(np.pi if direction<0 else 0)-x[2])
                v=direction*min(vb,2.8*d)*max(0.,np.cos(err))**2
                w=np.clip(3.5*err,-wb,wb)
            else:
                v=0.; w=np.clip(3.5*wrap(goal[2]-x[2]),-wb,wb)
            cmd=self.project(self.wmax*np.linalg.solve(self.B,[v,w]),prev,vb,wb,0 if direction<0 else direction)
            x=self.step(x,cmd); xs.append(x); us.append(cmd); prev=cmd
        return np.array(xs),np.array(us)

    def solve(self,x0,goal,v_cap=None,w_cap=None,forward_only=False):
        start=time.perf_counter(); x0=np.asarray(x0,float); goal=np.asarray(goal,float)
        x=x0[:5]; prev=x0[-2:]; vb=min(self.v_limit,v_cap) if v_cap and v_cap>0 else self.v_limit
        wb=min(self.w_limit,w_cap) if w_cap and w_cap>0 else self.w_limit
        dxy=goal[:2]-x[:2]; d=np.linalg.norm(dxy)
        bearing=np.arctan2(dxy[1],dxy[0]); front=abs(wrap(bearing-x[2]))+abs(wrap(goal[2]-bearing))
        back=abs(wrap(bearing+np.pi-x[2]))+abs(wrap(goal[2]-bearing-np.pi))
        changed=self.anchor is None or np.linalg.norm(goal[:2]-self.anchor[:2])>.03 or abs(wrap(goal[2]-self.anchor[2]))>.1
        desired=1 if forward_only or front<=back+.25 else -1
        if changed or forward_only or (d>.04 and desired!=self.direction and abs(wrap(bearing+(np.pi if self.direction<0 else 0)-x[2]))>2.2):
            self.us=None; self.direction=desired; self.anchor=goal.copy()
        # Only camera/ball actions forbid reversing. GoToPose may recover from overshoot.
        direction=1 if forward_only else 0
        prev=self.project(prev,prev,vb,wb,direction)
        xs,seed=self.reference(x,goal,prev,vb,wb,self.direction)
        # Use a reachable reference; when it is near the goal, use exact terminal pose.
        refs=xs[:,:3].copy()
        if np.linalg.norm(refs[-1,:2]-goal[:2])<.07:
            refs[-1,:2]=goal[:2]
            refs[-1,2]+=wrap(goal[2]-refs[-1,2])
        init=seed.copy()
        if self.us is not None:
            shifted=np.vstack([self.us[1:],self.us[-1:]])
            old=prev
            for k in range(self.steps):
                shifted[k]=self.project(shifted[k],old,vb,wb,direction); old=shifted[k]
            # Pick the lower-cost feasible seed, never publish a seed as a command.
            if float(self.fun((shifted/self.wmax).ravel(),x,refs.ravel(),prev/self.wmax)[0]) < float(self.fun((init/self.wmax).ravel(),x,refs.ravel(),prev/self.wmax)[0]):
                init=shifted
        low=np.r_[np.tile([0 if forward_only else -vb,-wb],self.steps),np.full(2*self.steps,-self.slew)]
        high=np.r_[np.tile([vb,wb],self.steps),np.full(2*self.steps,self.slew)]
        low[2*self.steps:2*self.steps+2]+=prev/self.wmax; high[2*self.steps:2*self.steps+2]+=prev/self.wmax
        def objective(z):
            f,g,_=self.fun(z,x,refs.ravel(),prev/self.wmax)
            return float(f),np.asarray(g).ravel()
        result=minimize(objective,(init/self.wmax).ravel(),jac=True,method='SLSQP',
                        bounds=Bounds(-1.,1.),constraints=[LinearConstraint(self.A,low,high)],
                        options={'maxiter':35,'ftol':1e-5})
        z=result.x; az=self.A@z
        violation=max(0.,np.max(low-az),np.max(az-high),np.max(np.abs(z))-1.)
        ok=bool(result.success and np.all(np.isfinite(z)) and np.isfinite(result.fun) and violation<1e-5)
        us=z.reshape(-1,2)*self.wmax
        pred=np.asarray(self.fun(z,x,refs.ravel(),prev/self.wmax)[2]).T
        if ok:self.us=us.copy()
        return {'u':tuple(us[0] if ok else np.zeros(2)), 'ok':ok,'status':int(result.status),
                'xs':pred,'us':us,'ms':(time.perf_counter()-start)*1000,'v_bound':vb,'w_bound':wb,
                'direction':self.direction,'residuals':[float(violation)],'iterations':int(result.nit),
                'message':str(result.message)}
