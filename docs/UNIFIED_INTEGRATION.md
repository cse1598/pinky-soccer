# Unified soccer integration (work in progress)

Base: attack-only game at a3e290df54ab2251f70a3079781f01ba5f91581d. Preserve first-to-three scoring, referee observation, manual START, PREPARE/READY, GOAL/OUT return/reset, STOP, sensor-only estimates and all physical safety checks. Existing attack defaults are unchanged.

## First import
- Calibration source and tests from f48c214c151a3205ce9b87d7dfe9b6c5b7f23f2a; combined README retains attack-side changes. Identification is an offline tool; never automatically apply unapproved estimates or change measured profiles.
- TVLQR core, controller and numerical tests from 07c2b2b2ce8b23e41d29ca3e26c5ecf32e486d6c; registered executable only. It is simulation-only and GoToPose-only, not a match-ready backend. Do not run it concurrently with the attack controller.
- Original branches and validation archives are retained. No raw bag or topics recording imported.

## Remaining work before main
1. Integrate the referee feature branch selectively. Compare messages, services, launch parameters and recorder; preserve attack-side match lifecycle and test compatibility. Do not overwrite the attack referee.
2. Restore optional TVLQR verifier launch path and compare it with existing MPC/Nav2. Reconcile shared launch/robot_stack/behavior_verifier files instead of replacing them. Confirm existing MPC/Nav2 functions and their supported environments. Expose only verified controller capabilities; TVLQR remains simulation-only until separately validated.
3. Make a single public game launch for the attack match, with default A* + geometric follower. Keep the old game launch as an internal stack or compatibility entry; avoid recursive includes. Separate game state and motion backend; adapt BehaviorGoal/MotionResult and streaming goals, contact handling, cancellation, collision/stop checks and exclusive cmd_vel ownership. Unsupported options must fail clearly, never silently fall back. Do not connect old controllers to live attack play before validating capabilities.
4. Run ROS 2 Jazzy colcon build and tests, existing attack rules/stop/restart/goal/out, per-backend GoToPose and safety regressions, and repeat two-robot MuJoCo first-to-three trials. The baseline itself has not completed a full first-to-three match; do not claim success without evidence. Gazebo attack play and real TVLQR remain unsupported until tested. Real hardware tests require user-controlled safe conditions and explicit approval.
5. Keep main unchanged until validated. Then fast-forward main or submit a reviewed PR; retain original branches until user approves cleanup. Preserve ignored experiment data separately.

Integration source import has not been built or tested on the user's PC.
