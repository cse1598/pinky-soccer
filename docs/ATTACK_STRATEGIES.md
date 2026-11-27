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
- Experimental strategy is MuJoCo only and requires working acados/CasADi setup.
  The original GoToPose solver/scheduler and default attack strategy remain unchanged.
- The optional approach wrapper uses the same ScheduledInput policy but is new and not
  yet validated. No ROS build, generated solver build, simulation match or timing test
  was performed when writing this change.

Validation: rebuild, run legacy-vs-legacy regression, then mixed strategy in both
team assignments. Check /<robot>/motion/status, /<robot>/motion/path, minimum separation,
collision count, approach time, solver deadlines, and the 3-goal match lifecycle.
An unaccepted solution must hold zero; investigate infeasible goals/solver timing before
real use. The existing A* collision checks and referee STOP retain priority.
