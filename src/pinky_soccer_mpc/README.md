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

## GoToPose: fixed-step minimum-time horizon search (method A)

GoToPose now uses `TimeOptimalMpc`: dt=0.1 s (10 Hz), candidate N=2..80,
with a 75 ms soft search budget and up to four local SLSQP attempts. N is the
primary objective. Each candidate jointly optimizes both wheel commands with
the existing wheel-lag model and command/slew/body bounds, terminal position
radius 12 mm, heading 2 deg, and terminal wheel speeds AND commands <=0.12
rad/s. Effort and centering break ties within N. The old geometric rollout
is only an initialization, not a tracked reference. Other actions still use
the existing controller. Actual completion still requires 2 cm/3 deg and
rest for 0.3 seconds; braking begins once the measured pose and rest criteria
are met. No model or simulator calibration was changed.

The search is approximate: local optimization, nearest continuous yaw lift,
bounded N and wall budget. It does not prove global minimum time. For a
distant/unresolved target, a converged fixed-horizon goal-only progress
solution may be used; it is NOT a terminal-feasible minimum-time solution.
If neither is available, the existing stop/failure logic applies. SLSQP
callbacks cannot preempt a native iteration, so 75 ms is not a hard guarantee.
Status/verification CSV report horizon_steps, planned_time, terminal_feasible,
attempts and lower_steps. Planned time excludes the actual 0.3 s dwell.
No new obstacle or field path constraints are added by this version. Validate
in simulation before real robot use. For consistent small-matrix timings
consider OPENBLAS_NUM_THREADS=1 when launching.

## acados backend comparison (GoToPose)

The default GoToPose solver is now `AcadosTimeOptimalMpc`, using acados SQP
and HPIPM. `mpc_backend:=slsqp` explicitly selects the previous implementation
for A/B comparison. Other actions remain unchanged. No silent solver fallback.
The same 0.1 s interval, candidate N=2..80, 75 ms soft search budget, wheel-lag
predictor, bounds, terminal tolerances and arrival dwell are retained to isolate
the optimizer change. The acados OCP includes two previous-command memory states;
its physical five-state prediction and objective are unchanged. A single
generated solver changes its horizon with set_new_time_steps(), without
compiling per candidate. First startup generates code in the solver cache.
The per-candidate SQP timeout is 15 ms at default settings; neither the native
timeout nor the outer search budget is a hard real-time guarantee.

A candidate is accepted only after replaying its inputs with the independent
NumPy predictor and checking all command/slew/body and applicable terminal bounds.
Statuses 2 or 7 are marked as nonconverged and accepted only if replay and
dynamics/inequality residuals pass for a terminal-feasible candidate. A
goal-progress fallback is also solved by acados. It requires status 0.
No raw seed is sent. This is still a local, bounded discrete-time search,
not a guarantee of global minimum time.

CSV records candidate statuses, replay violation, solve ID and age of the
optimization input at result receipt / command transmission in sim and wall
seconds. These are controller-side timing measurements, not physical motor
transport delay. The verifier also reports the maximum ground-truth position
error after first position entry, and maximum heading error after first joint
pose entry. Model lag, ramp and delay compensation are NOT changed in this
backend comparison. Validate in simulation before real hardware.

```bash
OPENBLAS_NUM_THREADS=1 python -m pytest -v -s src/pinky_soccer_mpc/test/test_acados_time_optimal.py
OPENBLAS_NUM_THREADS=1 ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco controller:=mpc mpc_backend:=acados verify:=true
# baseline: change mpc_backend:=slsqp
```

### acados elastic terminal convergence revision

The previous 35-iteration hard-terminal OCP failed four of seven movement
regressions. This revision uses normalized wheel states internally, linear
terminal projections forming an inscribed octagon, terminal-only elastic
slacks, a quadratic center penalty, ROBUST HPIPM, 100 SQP/QP iteration caps and
a ~21 ms default candidate timeout. The exact replay circle (12 mm),
2-degree yaw and 0.12 rad/s wheel and command limits remain unchanged.
Rejected terminal plans can initialize another candidate, never directly
command the robot. Path constraints remain hard. A nonconverged progress-only
candidate must independently satisfy path bounds, residual thresholds, and
improve the progress objective over a feasible braking rollout. Its
terminal_feasible flag stays false. Search budget remains soft, and
partial condensing horizon 2 stays unchanged to preserve resizing compatibility.
The generated solver cache key is bumped; JSON is stored in its code directory.
Each candidate now logs four KKT residuals, iterations, maximum slack,
terminal errors and acceptance. No calibration, delay, or motor-model changes.
Re-run the complete acados regression before simulation; the new elastic OCP
has not yet been compiled/executed on the target machine.

### Scheduled actuation and motor-profile prediction

GoToPose/acados now plans for the next 100 ms activation boundary, predicting
the current held wheel command for one period before solving. Early solutions
wait; a missing/late result is not reused at the following stage. The first
stage starts after one period; timing is in ROS/simulation time, with the
existing monotonic watchdog. The 50 Hz command heartbeat remains. A missed
slot brakes to zero; three consecutive missed/failed slots fail the skill.
This is compensation for asynchronous computation/application timing, not
identified network or sensor acquisition delay. The fused pose is still the
latest available estimate; receipt age and prediction errors are logged.
SLSQP and non-GoToPose actions retain their previous scheduling.

The acados OCP now has nine states (pose, actual wheels, ramped motor references,
previous commands). It uses the 5 ms profile_model symbolic map and an
independent scalar replay with matching numerical integration. Terminal
acceptance additionally requires near-zero ramped references. Actual arrival
tolerances and speed caps are unchanged. The profile rate and lag remain
configured assumptions, not newly identified dynamics. No arbitrary near-goal
speed reduction or extra damping weight has been added. The cache key changes.
New tests cover model parity, activation timing, and one-period delayed
closed-loop motion. Upstream acados deprecation warnings are not globally
suppressed; direct test access to deprecated N has been removed.

### Near-goal velocity damping (method A)

The primary criterion is still the shortest independently feasible integer
horizon found within the search budget. Within a candidate horizon, add a
smooth cost on physical forward and yaw velocities (not commanded velocity).
Defaults: radius 0.08 m, heading gate scale 25 degrees, normalized weights
q_v=0.15, q_w=0.60. Yaw damping is strongest near both target position and
orientation, so a necessary large in-place rotation is not penalized as
strongly. The radius is a continuous cost scale, not a hard speed limit.
Wheel, slew, terminal acceptance, motor profile and scheduled actuation are
unchanged. The progress-only fallback retains its prior objective.
`mpc_damping_scale:=0.0` disables the new residuals, `1.0` is default;
other nonnegative finite values scale both weights. This runtime OCP
parameter shares one generated solver across scales. Numerical behavior with
zero damping need not be bit-identical to the previous generated solver.
CSV includes damping_scale and the replayed stage damping_cost for each
selected solution. Neither the weight choice nor overshoot reduction is
validated on the real robot. No acados warnings are globally suppressed.

Run paired closed-loop numerical cases and both MuJoCo settings with the
same initial state, goal, ball location and OBS setting:
```bash
OPENBLAS_NUM_THREADS=1 python -m pytest -v -s src/pinky_soccer_mpc/test/test_damping.py src/pinky_soccer_mpc/test/test_scheduled_input.py src/pinky_soccer_mpc/test/test_acados_time_optimal.py
OPENBLAS_NUM_THREADS=1 ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco controller:=mpc mpc_backend:=acados mpc_damping_scale:=1.0 verify:=true
# baseline: mpc_damping_scale:=0.0
```


### Delay-aware wheel disturbance observer (experimental)

The experiment defaults to damping scale 0.0 and observer mode `off`.
Choose `mpc_observer_mode:=off|align|ndob` in game.launch.py (or sim_field /
real_game). All three use the existing scheduled 100 ms compute compensation.
`align` reconstructs the current pose/wheels using stamped feedback and
published command history; `ndob` additionally estimates lumped wheel
acceleration disturbances. Only GoToPose/acados uses the corrected prediction.
Off is the previous nominal scheduled model with zero new damping residual.

For each new wheel sample at source time t_k, replay the nonlinear ramp/lag
model with the actual published (driver-rounded) commands over the previous
sample interval. With h = t_k - t_{k-1}, the constant additive acceleration
sensitivity is g = tau*(1-exp(-h/tau)). Update
d_hat <- clip(d_hat + (1-exp(-h/0.12))*(innovation/g), -20, 20).
Units are rad/s^2. No numerical derivative or ground truth is used.
Samples out of order, more than 200 ms old, or materially in the future are
rejected; unstamped joint feedback uses receipt time and is flagged.
WorldModel.self.last_seen now records the TF/odom source stamp, not publication
time. Pose extrapolation is dead reckoning using the historical wheel states;
missing pose history leaves the pose uncorrected and is logged.
The next scheduled activation state uses the observer estimate; the solver
receives a transient forecast decaying exponentially with a 0.25 s time
constant between horizon stages. The disturbance is constant within a stage.
The independent replay uses the same forecast; terminal checks remain strict.
This is a pragmatic transient-disturbance forecast, not a proof of robustness
or guaranteed offset-free tracking under persistent disturbances. The existing
zero-command settling and measured arrival criterion are unchanged.
No actual motor-acceptance timestamp is available: input transport delay is
NOT explicitly identified or compensated, and real machines need synchronized
ROS clocks. Encoder-only NDOB cannot observe chassis slip. The gain, cap and
decay time are initial experimental choices, not identified values.

Compare off, align, ndob with damping 0, identical initial/target poses,
ball position and OBS settings. Inspect activation misses, raw and aligned
prediction error, timestamp ages, accepted/rejected measurements and d_hat.
Keep the real robot in off mode until validated at low speeds.

```bash
OPENBLAS_NUM_THREADS=1 python -m pytest -v -s src/pinky_soccer_mpc/test/test_delay_observer.py src/pinky_soccer_mpc/test/test_damping.py src/pinky_soccer_mpc/test/test_scheduled_input.py src/pinky_soccer_mpc/test/test_acados_time_optimal.py
OPENBLAS_NUM_THREADS=1 ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco controller:=mpc mpc_backend:=acados mpc_damping_scale:=0.0 mpc_observer_mode:=ndob verify:=true
```
