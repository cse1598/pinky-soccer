"""Fast independent scalar replay of profile_model's 5 ms numerical map."""
import math
import numpy as np


class RampPredictor:
    def __init__(self,params,dt):
        self.dt=float(dt)
        self.rl,self.rr,self.b=(params.m(k) for k in ('left_radius','right_radius','track'))
        self.tau_l,self.tau_r=params.m('tau_left'),params.m('tau_right')
        self.accel=float(params.limits['motor_accel'].value)

    def step(self,x,u,dt=None,disturbance=(0.,0.)):
        dt=self.dt if dt is None else float(dt)
        if dt<0 or not math.isfinite(dt):
            raise ValueError('invalid prediction interval')
        px,py,th,wl,wr,fl,fr=map(float,x)
        ul,ur=map(float,u)
        al,ar=map(float,disturbance)
        if dt==0:
            return np.asarray(x,float).copy()
        n=max(1,math.ceil(dt/.005))
        h=dt/n
        dl,dr=math.exp(-h/self.tau_l),math.exp(-h/self.tau_r)
        a=self.accel*h
        for _ in range(n):
            nl=fl+max(-a,min(a,ul-fl))
            nr=fr+max(-a,min(a,ur-fr))
            ql=dl*wl+(1-dl)*((fl+nl)/2+self.tau_l*al)
            qr=dr*wr+(1-dr)*((fr+nr)/2+self.tau_r*ar)
            l=self.rl*(wl+ql)/2
            r=self.rr*(wr+qr)/2
            v=(l+r)/2
            w=(r-l)/self.b
            mid=th+h*w/2
            px+=h*v*math.cos(mid)
            py+=h*v*math.sin(mid)
            th+=h*w
            wl,wr,fl,fr=ql,qr,nl,nr
        return np.array([px,py,th,wl,wr,fl,fr])
