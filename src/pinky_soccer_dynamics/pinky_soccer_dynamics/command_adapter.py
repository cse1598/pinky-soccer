"""
Wheel-speed targets <-> /cmd_vel for the velocity driver in use, and /joint_states -> wheels.

The NMPC decides wheel speed targets Omega_cmd (rad/s, positive = forward for that wheel).
The robot only accepts /cmd_vel, and its driver turns (v, w) back into wheel speeds with
*its own* radius and separation:

    Omega_L = (v - w * b_d / 2) / r_d,   Omega_R = (v + w * b_d / 2) / r_d

So the adapter inverts exactly that, with the driver parameters (not the calibrated model
geometry):

    v = r_d (Omega_R + Omega_L) / 2,     w = r_d (Omega_R - Omega_L) / b_d

The driver then reproduces the requested wheel speeds, and the measured effective radii
only enter the dynamics model. Using the model radii here would apply the calibration a
second time.

`driver_applied` reproduces what the driver really sends to the motors (ratio-preserving
100 rpm scaling and 0.229 rpm truncation on the real robot, separate v/w clamps in the
official MuJoCo bridge) so the controller can predict and log saturation.
"""

import math

RPM_TO_RAD = 2.0 * math.pi / 60.0


class CommandAdapter:
    """Driver-specific wheel <-> twist conversion."""

    def __init__(self, params):
        self.p = params
        self.r = params.d('wheel_radius')
        self.b = params.d('wheel_separation')
        self.max_rpm = params.d('max_rpm')
        self.rpm_resolution = params.d('rpm_resolution')
        self.max_linear = params.d('max_linear')
        self.max_angular = params.d('max_angular')

    def to_twist(self, omega_l, omega_r):
        """Return (linear.x, angular.z) that make the driver command these wheel speeds."""
        return (0.5 * self.r * (omega_r + omega_l), self.r * (omega_r - omega_l) / self.b)

    def driver_wheels(self, v, w):
        """Wheel speeds the driver derives from a twist, before its own limits."""
        return ((v - 0.5 * w * self.b) / self.r, (v + 0.5 * w * self.b) / self.r)

    def driver_applied(self, v, w):
        """Return (omega_l, omega_r, saturated) actually sent to the motors."""
        saturated = False
        if self.max_linear is not None and abs(v) > self.max_linear:
            v, saturated = math.copysign(self.max_linear, v), True
        if self.max_angular is not None and abs(w) > self.max_angular:
            w, saturated = math.copysign(self.max_angular, w), True
        wl, wr = self.driver_wheels(v, w)
        if self.max_rpm is not None:
            peak = max(abs(wl), abs(wr)) / RPM_TO_RAD
            if peak > self.max_rpm:
                scale = self.max_rpm / peak
                wl, wr, saturated = wl * scale, wr * scale, True
        if self.rpm_resolution:
            q = self.rpm_resolution
            # Dynamixel goal velocity is int(rpm / 0.229): truncation towards zero.
            wl = math.copysign(math.floor(abs(wl) / RPM_TO_RAD / q) * q * RPM_TO_RAD, wl)
            wr = math.copysign(math.floor(abs(wr) / RPM_TO_RAD / q) * q * RPM_TO_RAD, wr)
        return wl, wr, saturated

    def round_trip(self, omega_l, omega_r):
        """Wheel speeds that reach the motors for a wheel target (for logging/tests)."""
        v, w = self.to_twist(omega_l, omega_r)
        return self.driver_applied(v, w)


def wheels_from_joint_state(params, names, velocities, left='l_wheel_joint',
                            right='r_wheel_joint'):
    """Return (omega_l, omega_r) in the model convention from /joint_states fields."""
    vel = dict(zip(names, velocities))
    if left not in vel or right not in vel:
        raise KeyError(f'/joint_states lacks {left}/{right}')
    return params.sign_left * float(vel[left]), params.sign_right * float(vel[right])
