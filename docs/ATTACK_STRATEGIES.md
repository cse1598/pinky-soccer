# Selectable attack strategies (experimental)

The existing attack_only.xml, match preparation and referee lifecycle remain the default.
Each robot can choose its own policy at launch. With team:=Y (default), strategy selects
yellow and opponent_strategy selects blue. With team:=B the roles are reversed.

```bash
ros2 launch pinky_soccer_viz attack.launch.py mode:=mujoco robots:=2 \
    strategy:=path_mpc opponent_strategy:=legacy referee_ball_assist:=true
```

- legacy: original BT and geometric A* follower.
- path_mpc: copy of attack_only.xml, normal approach sends the direct behind-ball pose
  (no forced 20 cm side offset); original tactical stuck/flank recovery remains.
  A* produces a polyline. A waypoint rollout seeds the existing acados time-optimal
  horizon search; a 6 cm path corridor and moving opponent circles constrain the new
  solver. Candidate replay and activation check reject invalid paths. The existing
  AttackController remains the only cmd_vel owner, and retains the LiDAR/footprint
  stopping-envelope check. No geometric seed is used as a fallback input.
- Only GoBehindBall uses the experimental MPC. Spin, align, push, preparation, and
  recovery retain the existing geometric actions (with extra opponent prediction
  safety in the experimental stack).
- The prediction is constant velocity capped at 0.5 m/s over 1 second, then frozen;
  it cannot guarantee zero collisions. No spline smoothing yet: the corridor permits
  the optimizer to round corners. The goal is bounded-horizon minimum-time feasibility,
  not a proof of globally optimal time.
- Experimental strategy supports MuJoCo and explicit real-robot trials and requires working acados/CasADi setup on the PC.
  The original GoToPose solver/scheduler and default attack strategy remain unchanged.
- The optional approach wrapper uses the same ScheduledInput policy but is new and not
  yet validated in a complete match. On 2026-11-28, the real calibrated model generated
  successfully and accepted three obstacle-free fixed-input solves in 42–58 ms.
  Two used independently feasible nonconverged iterates; one converged. This is not
  a validation of runtime deadlines, dynamic obstacles, or real driving.

Real trials use one PC stack per namespace with `mode:=real robots:=1
robot_name:=yellow team:=Y strategy:=path_mpc referee_enabled:=false` (blue/B for
the other robot). Keep robot bringup and the shared referee running; stop the old
PC stack before replacing it. Calibration and motor identification arguments are
unchanged. `/yellow/motion/status.strategy` reports `path_mpc`; `backend` is
`path_mpc` during GoBehindBall and `geometric_path` for retained actions.

Both strategies enter the existing recovery after one second without motion
progress. A blocked-start axial escape uses the geometric follower (including
reverse when required), then returns to path MPC after the first escape waypoint.
The forward-only optimizer is not asked to execute a reverse escape leg. Sensor
or localization loss still stops motion; path recovery does not bypass validity.

Validation: rebuild, run legacy-vs-legacy regression, then mixed strategy in both
team assignments. Check /<robot>/motion/status, /<robot>/motion/path, minimum separation,
collision count, approach time, solver deadlines, and the 3-goal match lifecycle.
An unaccepted solution must hold zero; investigate infeasible goals/solver timing before
real use. The existing A* collision checks and referee STOP retain priority.

## Compact approach targets (2026-11-28)

AttackBall defaults: approach_offset_m=0.13 (was0.15), orbit_clearance_m=0.14
(was0.20), recovery_clearance_m=0.14 (was0.18), in metres. The normal legacy
side waypoint and the stagnant-ball opposite flank use these smaller targets.
Path MPC still delegates its normal detour to A*, using the closer behind-ball
target. The existing body-cover radius plus margin and ball radius is about
0.12587m; obstacle inflation, exact-footprint checks and MPC constraints remain
unchanged. These are tactical targets, not a guaranteed minimum travelled path.
BT regression includes mirrored teams and both planner policies; all3 CTest
groups passed, including18 tree tests. No additional MuJoCo or real driving was
performed for this adjustment. Restart the PC game launch to load the new BT.

All red LED ERROR entry conditions are listed in [RED_ERROR_CONDITIONS.md](RED_ERROR_CONDITIONS.md).
User policy9 now waits in RECOVERING and resumes after trusted localization returns;
other ERROR policies are retained. Path stalls prefer a feedback-measured 0.10m
reverse escape, then replan; a blocked/unsuccessful rear escape uses alternative
checked motions. Operator/referee STOP cancels automatic resume.

## Current legacy checkpoint (2026-11-28)

At the user's request, legacy is restored to the measured 10cm reverse/replan
recovery configuration. The compact 13/14cm approach targets above remain.
Later normal-approach reverse selection, CarryBall, contest flank maneuvers,
wall-specific shot axes, ball-centred push steering, A* lookahead and curved
approach travel have been removed. Normal approach uses the original geometric
follower; explicit Backoff and blocked-start reverse escape remain available.
Localization RECOVERING, referee STOP, PREPARE UI fixes, odometry heading selection,
and existing calibration/motor parameters are preserved. Red ERROR is not globally
suppressed: the earlier recoverable localization/path cases retain their recovery.

Launch commands and arguments are unchanged: select `strategy:=legacy` for each
robot. Restart the PC game launches after rebuilding; this rollback does not
require robot-side source deployment. No DDS or network settings were changed.

Validation: 43 targeted Python tests passed, all three BT CTest groups passed,
and BT/viz packages rebuilt successfully. No new simulated match or real driving
was performed. The [six-match report](validation/evolve_20261128/README.md)
describes the later implementation that was rolled back, not this checkpoint.
