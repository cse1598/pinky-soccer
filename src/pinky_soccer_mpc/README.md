# Pinky Pro NMPC

## Motor-profile v1

Point-to-pose acados solver. Physical states: pose, wheel speeds, and two
internal motor-profile speeds. Previous wheel commands add two bookkeeping
states for input-rate cost (9 states total). Discrete map: 50 ms stages, 5 ms
internal substeps. Driver profile acceleration: 74.9 rad/s². NMPC ordinary
command slew: 67.4 rad/s²; command speed: 92.7 RPM (90% of 103 RPM at 5 V).
Wheel load response is a first-order lag, initially 0.05 s **assumption**,
not identified. No explicit lateral slip/dead-time compensation yet.

- 2.0 s / 40 steps; one warm-start SQP solve per tick. Travel direction
  chosen once per substantially changed goal, comparing front/back bearings
  and final heading. Ball actions forward-only. No periodic candidate switching.
- Costs: distance, time-to-go surrogate, travel/final heading, normalized
  wheel acceleration, actual command increments and angular command increments.
- Soft path caps for feasibility; transmitted speed/direction projected onto
  current caps. Slew may yield to stricter caps or a safety stop.
- Status 2 requires finite solution and bounded equality/inequality residuals.
  Invalid results or stale requested wheel feedback stop immediately.
- Arrival brakes translation, aligns, sends zero inside heading tolerance,
  holds once wheels settle. Real safety caps unchanged (0.08 m/s, 0.5 rad/s).
- Default simulation angular cap 2 rad/s retained to protect AMCL.

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true
ros2 launch pinky_soccer_viz game.launch.py mode:=gazebo verify:=true
```

CSV verifier adds MPC state/status/time/direction, wheel feedback, internal
profile, wheel command, period and bounds. For rigorous identification, record
synchronized native-rate joint states, odometry and command timestamps.
`check_acados` checks the legacy five-state toolchain; `test_pose_mpc.py`
exercises the new model. `test_profile_model.py` checks symbolic and numeric
dynamics and command caps.

Existing single-lag identification may absorb the motor ramp into tau.
Do not use its parameters as the new two-stage model's wheel lag without
re-identification. Nonzero delay requires explicit compensation.
This revision is not yet tuned against game logs.
