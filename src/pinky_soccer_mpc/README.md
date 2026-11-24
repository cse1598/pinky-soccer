# Pinky Pro NMPC — Phase 1

Only GoToPose is under active skill verification. The BT submits a BehaviorGoal
with a unique goal_id and waits for the matching MotionResult. The controller
owns arrival, failure and cancellation. No separate P alignment or BT pose
tolerance. A zero command is used for braking/settling and safety.

- 5-state smooth wheel-lag model, CasADi gradient, SciPy SLSQP direct shooting.
  The prior 9-state acados experiment is retained in experimental_acados_pose.py.
- 2 seconds / 20 stages, solve at 10 Hz in one worker, command at 50 Hz.
- Feasible geometric reference and warm start are **not** published as commands.
  Only accepted optimized commands are sent. Hard wheel, twist and command
  increment bounds. Actual acceleration depends on motor physics.
- Default 92.7 RPM, simulation angular cap 2 rad/s, real safety caps unchanged.
- Success: position <=2 cm, heading <=3 degrees, wheel and linear/angular speed
  thresholds and fresh measured feedback for 0.3 seconds (estimated pose).
  Simulator ground truth is for evaluation, not feedback.
- No-progress and repeated optimizer failure report FAILURE. Stale input stops.
- Legacy single-lag identification is not automatically applied; dynamics
  calibration and measured lag/delay remain separate work. Existing odometry
  profiles are retained. Real GoToPose requires verified wheel signs and
  wheel_feedback:=true.
- Reactive controller is retained for legacy comparison, not the GoToPose
  result protocol.

In the workspace virtual environment install CasADi and SciPy if missing:
```bash
python -m pip install "numpy<2" "scipy<1.17" casadi
```
After a full build (message interfaces changed):
```bash
python -m pytest -q src/pinky_soccer_mpc/test/test_pose_mpc.py
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco controller:=mpc verify:=true
```
CSV includes goal ID, backend, optimizer status/reason, iterations, position
and heading error, wheel and command data. The status code is now SLSQP,
not acados. ROS/Gazebo/MuJoCo integrated runtime validation is pending.
