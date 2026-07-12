# Report templates by tier

Copy the matching template into the PR body **Verification** section or the
final chat summary.

---

## T0 — one sentence

```markdown
## Verification
Bumped `planning_timeout` default in `dubins_race.yml` (30→35 s); `bash scripts/check/formatting.sh` and `bash scripts/check/types.sh` pass.
```

---

## T1 — bullets

```markdown
## Verification
- **Problem:** `mypy` missing `Any` import in `planner_node_ros.py`
- **Fix:** added `from typing import Any`
- **Commands:** `bash scripts/check/types.sh` → exit 0
```

---

## T2 — short sections

```markdown
## Problem
PPP `render-ppp` job timed out at 36 s with empty `qpos_history` on CI.

## Fix
Raise physics showcase `planning_timeout` to 90 s in `render_mujoco.py` only.

## Verification
- `bash scripts/check/formatting.sh` — pass
- `bash scripts/check/types.sh` — pass
- `python3 -m pytest tests/simulation/test_render_mujoco.py -k ppp` — 12 passed
```

---

## T3 — evidence report

```markdown
## Problem (verbatim)
CI `render-dubins` exit 1:
`RuntimeError: Dubins race simulation failed before both agents reached goal
(race_duration_s=300.0, max_cross_track_error_m=67.80)`
Run: https://github.com/org/repo/actions/runs/NNNNNN

## Timeline (sourced)
| Time | Event | Source |
| --- | --- | --- |
| t=0 | Planning starts (unseeded ARC) | local repro |
| t≈50s | Agents stall, x≈42 m | pose log |
| t=300s | Race timeout, `both_reached_goal=False` | runner result |

## Analysis (interpretive)
Unseeded planner RNG produces untrackable paths under physics (~25% fail).

## Fix
Pin `planner_rng_seed=11` during physics showcase export.

## Verification
| Trial | Unseeded | Seeded |
| --- | --- | --- |
| 8 runs | 6/8 pass (75%) | 8/8 pass (100%) |

![success rate chart](artifacts/03_failure_rate_bars.png)

## Recurrence
Prior fix (`8b2cf32`) correctly rejected fake clips but exposed underlying flake.
```

---

## T4 — engineer-grade (full)

Use all sections from T3 plus:

```markdown
## Mechanism (export transform)
`_resample_pose_history(full_300s_log, 1050_frames)` maps viewer 35 s → sim 300 s
(8.6× apparent speedup). Honest gate: reject when `not both_reached_goal`.

## Evidence layers
1. CI log verbatim (above)
2. Control variable: same physics, only RNG seed changes
3. XY trajectories + x(t) plot + viewer-vs-sim clock plot
4. MuJoCo overview snapshots from `scripts/video.sh` pipeline
5. Fix chain diagram

## Artifacts
- `/opt/cursor/artifacts/<topic>/01_trajectory_xy_comparison.png`
- `/opt/cursor/artifacts/<topic>/02_time_compression_manipulation.png`
- Repro: `python3 scripts/evidence_<topic>.py`

## Follow-ups
| Action | Owner | Verb | Outcome | Tracker | Due |
| --- | --- | --- | --- | --- | --- |
| Merge seed fix | agent | add | CI render-dubins green | PR #NNN | — |
| Monitor v1.2.2 tag release | maintainer | verify | both showcase jobs pass | release.yml | tag day |

## What went well
- Honest failure gate prevented shipping misleading showcase clips.
```
