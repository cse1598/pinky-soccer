"""Experimental corridor extension of the existing minimum-time acados solver."""
import hashlib
import math
from dataclasses import replace

import numpy as np

from .acados_setup import cache_dir
from .avoidance_mpc import AvoidanceTimeOptimalMpc, CIRCLE_SLOTS, SLOT
from .collision_geometry import interpolate_poses
from .obstacle_snapshot import circle_clearance, wall_clearance

SEGMENTS = 12
WIDTH = .06
PREDICT_SECONDS = 1.0


def corridor_distance(poses, path):
    p = np.asarray(poses, float)[:, None, :2]
    path = np.asarray(path, float)
    a, b = path[:-1], path[1:]
    d = b-a
    t = np.clip(np.sum((p-a)*d, axis=-1)/np.maximum(np.sum(d*d, axis=-1), 1e-12), 0., 1.)
    return np.min(np.linalg.norm(p-a-t[..., None]*d, axis=-1), axis=1)


class PredictedScene:
    """Constant velocity for one second, then hold; not an adversarial safety guarantee."""
    def __init__(self, snapshot, velocities):
        self.snapshot = snapshot
        self.velocities = np.asarray(velocities, float).reshape(-1, 2)
        if len(self.velocities) != len(snapshot.circles):
            raise ValueError('velocity/circle mismatch')

    def at(self, t):
        t = max(0., min(float(t), PREDICT_SECONDS))
        return replace(self.snapshot, circles=[
            replace(c, x=c.x+v[0]*t, y=c.y+v[1]*t,
                    radius=c.radius+(.035*t if c.kind=='opponent' else 0.))
            for c,v in zip(self.snapshot.circles,self.velocities)])

    def clearances(self, poses, times):
        times = np.clip(np.asarray(times,float), 0., PREDICT_SECONDS)
        cols = []
        if self.snapshot.walls is not None:
            cols.append(wall_clearance(poses, self.snapshot.walls))
        for c,v in zip(self.snapshot.circles,self.velocities):
            cols.append(circle_clearance(poses, (c.x+v[0]*times,c.y+v[1]*times))
                        -c.radius-(.035*times if c.kind=='opponent' else 0.))
        return np.column_stack(cols) if cols else np.zeros((len(poses),0))


class PathTimeOptimalMpc(AvoidanceTimeOptimalMpc):
    def __init__(self, *args, **kwargs):
        self.route = np.zeros((2,2))
        self.scene = None
        self.lead = 0.
        super().__init__(*args, **kwargs)

    def build_ocp(self):
        import casadi as ca
        ocp = super().build_ocp()
        model = ocp.model
        p = ca.SX.sym('corridor', 4*SEGMENTS)
        xy = model.x[:2]
        ds = []
        for i in range(SEGMENTS):
            a,b = p[4*i:4*i+2],p[4*i+2:4*i+4]
            d=b-a
            t=ca.fmin(1.,ca.fmax(0.,ca.dot(xy-a,d)/ca.fmax(1e-12,ca.dot(d,d))))
            ds.append(ca.sumsqr(xy-a-t*d)/(WIDTH*WIDTH))
        h=ds[0]
        for d in ds[1:]:
            h=ca.fmin(h,d)
        model.p=ca.vertcat(model.p,p)
        model.con_h_expr=ca.vertcat(model.con_h_expr,h)
        model.con_h_expr_e=ca.vertcat(model.con_h_expr_e,h)
        c=ocp.constraints
        c.lh,c.uh=np.r_[c.lh,-1.],np.r_[c.uh,1.]
        c.lh_e,c.uh_e=np.r_[c.lh_e,-1.],np.r_[c.uh_e,1.]
        ocp.parameter_values=np.r_[ocp.parameter_values,np.zeros(4*SEGMENTS)]
        model.name='pinky_corridor_'+hashlib.sha256(
            (model.name+'-path-v1-'+str(SEGMENTS)+'-'+str(WIDTH)).encode()).hexdigest()[:12]
        ocp.code_gen_options.code_export_directory=str(cache_dir(f'c_generated_code_{model.name}'))
        ocp.code_gen_options.json_file=f'{model.name}.json'
        return ocp

    def stage_parameters(self,k,p):
        out=super().stage_parameters(k,p).copy()
        snap=self.scene.at(self.lead+k*self.dt)
        start=len(out)-SLOT*CIRCLE_SLOTS
        for i,c in enumerate(snap.circles[:CIRCLE_SLOTS]):
            out[start+SLOT*i:start+SLOT*(i+1)]=(
                c.x,c.y,c.radius+snap.config.margin+snap.config.solver_margin,1.)
        seg=np.column_stack((self.route[:-1],self.route[1:]))
        pad=np.tile(np.r_[self.route[-1],self.route[-1]],(SEGMENTS-len(seg),1))
        return np.r_[out,np.vstack((seg,pad)).ravel()]

    def stage_bounds(self,k,lo,hi):
        a,b=super().stage_bounds(k,lo,hi)
        return (a,b) if k==0 else (np.r_[a,-1.],np.r_[b,1.])

    def collision_violation(self,xs):
        poses=interpolate_poses(xs)
        times=self.lead+np.arange(len(poses))*self.dt/5
        clear=self.scene.clearances(poses,times)
        violation=max(0.,float(np.max(corridor_distance(poses,self.route)-WIDTH)))
        if clear.size and self.required is not None:
            violation=max(violation,float(np.max(self.required[None,:]-clear)))
        return violation

    def seed(self,n,x,goal,prev,vb,wb,forward,direction):
        # Waypoint rollout is initialization only; never transmit it as a command.
        x=np.asarray(x,float).copy()
        prev=np.asarray(prev,float).copy()
        i=1
        us=[]
        for k in range(n):
            while i<len(self.route)-1 and np.linalg.norm(x[:2]-self.route[i])<.06:
                i+=1
            d=self.route[i]-x[:2]
            distance=np.linalg.norm(goal[:2]-x[:2])
            if i==len(self.route)-1 and distance<.018:
                v=0.
                w=3.5*math.atan2(math.sin(goal[2]-x[2]),math.cos(goal[2]-x[2]))
            else:
                e=math.atan2(d[1],d[0])-x[2]
                e=math.atan2(math.sin(e),math.cos(e))
                v=min(vb,2.8*distance)*max(0.,math.cos(e))**2
                w=3.5*e
            u=self.project(self.wmax*np.linalg.solve(self.B,[v,np.clip(w,-wb,wb)]),
                           prev,vb,wb,1 if forward else 0)
            us.append(u)
            x=self.predictor.step(x,u)
            prev=u
        return np.asarray(us)

    def solve_path(self,x0,goal,route,scene,v_cap,w_cap,lead):
        route=np.asarray(route,float)
        if route.ndim!=2 or route.shape[1]!=2 or not 2<=len(route)<=SEGMENTS+1:
            raise ValueError('unsupported path size')
        if not np.isfinite(route).all():
            raise ValueError('invalid path')
        self.route,self.scene,self.lead=route,scene,float(lead)
        return super().solve(x0,goal,v_cap,w_cap,forward_only=True,
                             obstacles=scene.at(lead))
