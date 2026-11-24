"""Pure scheduled activation policy for the asynchronous 10 Hz MPC worker."""
import math


class ScheduledInput:
    """One input per interval, with a fixed one-period lookahead.

    A late result is never applied retroactively. The ROS clock is used for
    prediction/activation and monotonic wall time for the worker watchdog.
    """
    def __init__(self,dt,tolerance):
        self.dt=float(dt)
        self.tolerance=float(tolerance)
        if not (0<self.tolerance<self.dt):
            raise ValueError('invalid activation tolerance')
        self.clear()

    def clear(self):
        self.due=None
        self.ready=None

    def begin(self,now):
        self.due=now+self.dt
        self.ready=None
        return self.due

    def offer(self,now,result):
        if self.due is None or now>self.due+self.tolerance+1e-9:
            return False
        self.ready=result
        return True

    def take(self,now):
        if self.due is None or now+1e-8<self.due:
            return False,None,None
        error=now-self.due
        result=self.ready if error<=self.tolerance+1e-9 else None
        self.clear()
        return True,result,error
