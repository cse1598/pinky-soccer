"""Near-goal velocity damping; no hard speed cap or separate pose controller."""
import math

import numpy as np

RADIUS_M = .08
HEADING_RADIUS_RAD = math.radians(25.)
Q_V = .15
Q_W = .60


def weights(distance_sq,heading_error):
    """Smooth position gate and periodic heading gate (scalar or array)."""
    position=RADIUS_M**2/(distance_sq+RADIUS_M**2)
    heading=HEADING_RADIUS_RAD**2/(HEADING_RADIUS_RAD**2+
                                   2*(1-np.cos(heading_error)))
    return position,position*heading


def symbolic_residual(error,vw,v_cap,w_cap,amplitude):
    """Nonlinear least squares residuals; 0.5*r.T*r equals damping cost."""
    import casadi as ca
    pos=RADIUS_M**2/(ca.sumsqr(error[:2])+RADIUS_M**2)
    heading=HEADING_RADIUS_RAD**2/(HEADING_RADIUS_RAD**2+
                                  2*(1-ca.cos(error[2])))
    return amplitude*ca.vertcat(
        ca.sqrt(2*Q_V*pos)*vw[0]/v_cap,
        ca.sqrt(2*Q_W*pos*heading)*vw[1]/w_cap)


def trajectory_cost(xs,goal,B,wmax,dt,v_cap,w_cap,scale):
    """Independent replay cost for the N stage-start measured wheel states."""
    xs=np.asarray(xs,float)[:-1]
    error=xs[:,:3]-np.asarray(goal,float)
    vw=(xs[:,3:5]/wmax)@B.T
    gv,gw=weights(np.sum(error[:,:2]**2,axis=1),error[:,2])
    return float(scale*dt*np.sum(Q_V*gv*(vw[:,0]/v_cap)**2+
                                 Q_W*gw*(vw[:,1]/w_cap)**2))
