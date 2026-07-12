# Execution comparison validation report

**Tier:** T4 — chronic visual simulation failure, prior misleading artifact,
cross-module capture changes, and explicit maintainer distrust.

## Verdict

The current comparison is valid for the acceptance requirements in
[`execution-comparison-requirements.md`](execution-comparison-requirements.md):
it proves upper-box contact, unobstructed comparable views, two native blue
hand arrows, and two real force traces from one Gazebo run.

It is **not** a pixel-identical reproduction of the 2015 desktop. The original
used Gazebo Classic plus CoDyCo `StageTestTasks`; this repository now uses
Gazebo Harmonic and a joint-position approximation. If pixel-identical joint
poses or the original desktop chrome are requirements, this result must be
rejected until the CoDyCo controller is restored.

## Problem (maintainer feedback)

The previous comparison was rejected for four recurring reasons:

1. fake post-processed arrows had previously been presented as simulation output;
2. the robot camera was cropped or covered by plot windows;
3. both arrows were not simultaneously visible in the side camera;
4. plots and screenshots were asserted as proof without executable acceptance criteria.

## Timeline (sourced)

| Event | Source |
| --- | --- |
| Fake overlay script removed and headless geometry introduced | Git history, PR #27 |
| A no-regression verifier was added for contacts, arrows, framing, and plots | `scripts/verify_no_regression.py` |
| Maintainer reported the robot was still covered by other windows | PR #27 conversation |
| Camera and plots were separated into non-overlapping panels | `scripts/capture_comparison.sh` |
| Maintainer reported the print screen still did not match requirements | PR #27 conversation |
| Requirements were written before the next capture | `docs/execution-comparison-requirements.md` |
| One side camera and one arrow-evidence camera were captured in the same run | `assets/worlds/release_demo.world`, capture log |
| Requirements-gated capture exited 0 with `NO-REGRESSION PASS` | `scripts/capture_comparison.sh` output |

## Analysis

### Why one camera was insufficient

The 2015 screenshot uses a side profile. That view makes the robot / box
composition comparable, but the two hands overlap in image space, so one arrow
can occlude the other. Rotating to a front-oblique view separates both arrows,
but no longer matches the reference composition.

The correct evidence design is therefore two labeled, simultaneous raw views:

- `/release_camera`: side profile for pose and scene comparison;
- `/arrow_camera`: front-oblique view for two-hand arrow visibility.

This is not duplication or image fabrication: both are Gazebo camera sensors in
the same world, captured at the same interaction state.

### Why the plots are valid

Each plot process subscribes directly to one `/vectorview/...` contact topic
before animation begins. It samples the latest real contact force at 25 Hz and
draws original and filtered magnitudes. Missing or all-zero traces terminate the
capture instead of creating a placeholder graph.

## Fix chain

```text
single obstructed / ambiguous screenshot
→ cannot prove pose and two arrows simultaneously
→ separate camera and plot rectangles
→ side camera proves scene parity
→ arrow camera proves two visible native arrows
→ raw contact messages prove both upper-box contacts
→ transport subscriptions prove real force traces
→ verifier rejects any missing layer
```

## Requirements verification

| ID | Result | Evidence |
| --- | --- | --- |
| V1 | PASS | Side-frame foreground bounds `(242,115)–(878,693)`; full robot and boxes fit. |
| V2 | PASS | Camera row occupies `y=0..499`; plot row starts at `y=505`; no overlap. |
| V3 | PASS | Left contact `(0.200, +0.190, 0.474) m`; right `(0.200, -0.190, 0.474) m`. |
| V4 | PASS | Arrow frame has 1,374 blue pixels in two separated regions (476 / 859 pixels). |
| V5 | PASS | Current panel labels side and arrow-evidence views independently. |
| S1 | PASS | Both raw messages name `object::main::collision`. |
| S2 | PASS | Blue arrows exist in raw `/arrow_camera`; capture has no overlay-script reference. |
| S3 | PASS | Upper object is static and visibly stacked on the base. |
| D1 | PASS | One plot per hand from the same capture process. |
| D2 | PASS | 299 samples per hand; 293 non-zero samples; peaks 353 N / 322 N. |
| D3 | PASS | Both frames, contacts, and plots are produced before the single sim process is stopped. |
| I1 | PASS | Zero red pixels in both raw frames; raw SHA-256 values recorded. |
| I2 | PASS | Missing contact, arrow, trace, or complete camera view is a terminating verifier assertion. |
| I3 | PASS | Both raw PNGs and the JSON report are committed separately. |

Machine-readable details:
[`no-regression-report.json`](no-regression-report.json).

## Evidence layers

1. **Reproduce:** `scripts/capture_comparison.sh`
2. **Control:** one Gazebo world and animation; only camera viewpoint differs
3. **State / time:** raw contact positions and 12-second force traces
4. **Pixel proof:** unmodified side and arrow camera PNGs
5. **Fix chain:** documented above; verifier executes before artifacts are accepted

## Artifacts

- [`images/execution_comparison.png`](images/execution_comparison.png)
- [`images/execution_current_raw.png`](images/execution_current_raw.png)
- [`images/execution_arrows_raw.png`](images/execution_arrows_raw.png)
- [`no-regression-report.json`](no-regression-report.json)

## Commands and results

```bash
cmake --build build
# result: vector-view, vector-gui, and test targets built

scripts/capture_comparison.sh
# result: exit 0
# verdict: NO-REGRESSION PASS

cd build && ctest --output-on-failure
# result: 2/2 tests passed
```

## Recurrence

This failure mode has recurred across multiple iterations:

- **First wrong fix:** paint arrows in post-processing.
- **Second incomplete proof:** real arrows, but ambiguous / obstructed framing.
- **Third incomplete proof:** executable metrics without an explicit visual contract.
- **Current fix:** written requirements + two raw synchronized views + real plots +
  terminating verification.

The current fix addresses the evidence mechanism, not only the screenshot.

## Remaining limitation

| Owner | Verb | Measurable outcome | Tracker | Due |
| --- | --- | --- | --- | --- |
| Maintainer / future agent | restore | Run the original CoDyCo `StageTestTasks` controller and match 2015 joint trajectories if pixel-identical pose parity is required | follow-up issue / PR | before claiming pixel identity |
