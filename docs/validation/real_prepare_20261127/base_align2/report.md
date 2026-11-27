# GoToPose offline evidence

Root cause: **not established automatically**. Input context and completeness matter.

All plots use recorder receipt time. Source clocks are not shifted or interpolated.

| Topic role | Samples | Mean Hz | Receipt p95 ms | Source p95 ms |
|---|---:|---:|---:|---:|
| status | 3509 | 49.9433052148504 | 24.441061750008686 | None |
| cmd | 1570 | 22.350370268473153 | 51.711611000000346 | None |
| goal | 1408 | 20.0426827044954 | 52.512732000000106 | 51.7512983 |
| joints | 1401 | 20.003268556943084 | 62.83034845000124 | 50.420858 |
| odom | 2103 | 30.0302345441448 | 47.25540835000146 | 33.46883635 |
| raw_joints | 2103 | 30.03470793602707 | 47.2021339999987 | 33.46883635 |
| drive | 1402 | 20.0157374491463 | 63.69570700000082 | None |
| tf | 1451 | 46.88506691602787 | 53.252639800001944 | 53.44919555 |
| world | 928 | 30.000050298142583 | 40.47963299999892 | 38.875864 |
| result | 172 | 49.01461013453771 | 29.261147500001528 | 29.2715735 |

Missing roles: amcl

## Interpretation limits

- No root cause is established automatically; use time-aligned evidence and trial context.
- A missed flag is not a unique activation event. True-run count is only a lower bound.
- Completed-solve metrics omit canceled generations: short observed solves do not rule out deadline misses.
- Receipt minus source stamp includes clock offset and buffering, not just sensor/network delay.
- AMCL/TF/world steps can be legitimate localization corrections; wheel odom is not independent ground truth.
- Low commanded speed with zero wheel feedback is a candidate deadzone/stiction observation, not proof.
- Real bringup merges wheel feedback through joint_state_publisher at 20 Hz and restamps it. Record raw_joints to assess physical measurement freshness.

## Timing evidence

```json
{
  "observed_completed_solves": 25,
  "missed_flag_true_runs_lower_bound": 0,
  "exact_missed_slot_count": null,
  "reason": "activation_missed is sticky; canceled generation results have no completion metrics",
  "metrics": {
    "ms": {
      "n": 25,
      "min": 2.3714210001344327,
      "median": 27.386087000195403,
      "p95": 73.84005940002679,
      "max": 79.12210600079561
    },
    "acados_ms": {
      "n": 25,
      "min": 0.182,
      "median": 15.950000000000001,
      "p95": 31.65639999999999,
      "max": 56.693
    },
    "worker_wait_ms": {
      "n": 25,
      "min": 1.4819640000496292,
      "median": 3.688771999804885,
      "p95": 24.871961600365463,
      "max": 34.8779679989093
    },
    "result_pickup_ms": {
      "n": 25,
      "min": 1.1937680010305485,
      "median": 10.956928999803495,
      "p95": 17.68020380077359,
      "max": 32.9062349992455
    },
    "solve_input_age_wall_s": {
      "n": 25,
      "min": 0.01891363899994758,
      "median": 0.04889724100030435,
      "p95": 0.09708727879915385,
      "max": 0.10256329599906167
    },
    "solve_input_age_sim_s": {
      "n": 25,
      "min": 0.0189058780670166,
      "median": 0.04889678955078125,
      "p95": 0.09708313941955565,
      "max": 0.10256218910217285
    }
  },
  "status_reasons": {
    "stale input": 1967,
    "STOP goal": 1370,
    "waiting for scheduled solution": 17,
    "scheduled optimized input": 113,
    "waiting for rest": 41,
    "settled": 1
  },
  "sticky_jitter_status_weighted_ms": {
    "n": 166,
    "min": 0.0858306884765625,
    "median": 0.47397613525390625,
    "p95": 20.19047737121582,
    "max": 20.339012145996094
  }
}
```

## Zero-command onset associations (not causality)

```json
[
  {
    "t": 0.019870293,
    "status_state": "stop",
    "reason": "stale input",
    "activation_missed": null,
    "goal_id": "0"
  },
  {
    "t": 38.308923787,
    "status_state": "stop",
    "reason": "stale input",
    "activation_missed": null,
    "goal_id": "0"
  },
  {
    "t": 40.311504385,
    "status_state": "waiting",
    "reason": "waiting for scheduled solution",
    "activation_missed": false,
    "goal_id": "1795776749317004399"
  },
  {
    "t": 42.076180604,
    "status_state": "settle",
    "reason": "waiting for rest",
    "activation_missed": false,
    "goal_id": "1795776749317004399"
  }
]
```

All events are retained in summary.json; normalized raw data are in samples.csv.
