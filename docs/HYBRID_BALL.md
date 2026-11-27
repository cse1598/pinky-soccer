# Referee-first hybrid ball sensor

Default attack.launch.py ball_source:=hybrid. Existing robot, assist, referee and auto
modes are retained. The local camera is selected immediately when a calibrated
metric observation with confidence >= 0.65 agrees with the referee within 0.06 m
(after compensating capture-time difference using referee velocity). No dwell,
consecutive-frame condition, or 0.5-second source-switch delay. Existing packet
freshness limits (referee 0.4 s, local BallFilter observed_timeout) only reject old
observations; a negative detection invalidates the corresponding observation immediately.

One continuous 4-state Kalman filter processes distinct selected position observations
and fresh referee velocity; correction magnitude is bounded, with Joseph-form
covariance. This is a heuristic robust filter, not a calibrated uncertainty model.
A source change does not reset the state. If the referee has no valid observation,
the estimator predicts the last accepted state rather than accepting an unverified
local point; ordinary LOST/reacquisition semantics apply. Large real ball jumps
may take longer to converge. No extra moving-average delay.

Both real and MuJoCo attack launches now pass field dimensions from the selected
field YAML to the referee. The nominal referee_field/map alignment is still assumed;
no measured SE(2) correction is fabricated. Local camera position is transformed
using capture-time TF. Self localization is unchanged. Actual disagreement thresholds
and uncertainty need tuning using simultaneous observation logs.

Diagnostics: /<robot>/world_model/ball_fusion (source, reason, difference_m,
capture timestamps, confidence, filtered state). The original PointStamped and
ball_detection/status remain available. Hybrid uses the metric point and confidence
in the same status message to avoid cross-topic race.

Pure tests added. ROS launch, camera streams, simulation and real matches require
validation on the user's system.
