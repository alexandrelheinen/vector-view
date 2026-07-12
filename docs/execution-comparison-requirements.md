# Execution comparison acceptance requirements

The comparison is accepted only when it demonstrates equivalent **behavior**,
not merely a similar-looking image.

## Visual requirements

| ID | Requirement | Acceptance evidence |
| --- | --- | --- |
| V1 | The primary current view uses a side profile comparable to the 2015 Gazebo Classic view: robot left of two stacked boxes, full body visible. | Unmodified `/release_camera` frame. |
| V2 | No plot, title, terminal, or other window covers the current robot or boxes. | Camera panels and plot panels occupy separate rectangles. |
| V3 | Both hands press opposite side faces of the upper box at approximately the same height. | Left contact `y > 0.15 m`, right contact `y < -0.15 m`, both `0.44 m < z < 0.50 m`. |
| V4 | Two **blue** force arrows are visibly separated and anchored at the hands. | Unmodified `/arrow_camera` frame contains two substantial blue regions. |
| V5 | The comparison labels the side view and arrow-evidence view; neither is represented as the 2015 frame. | Labels embedded in the layout outside robot / box pixels. |

## Simulation and data requirements

| ID | Requirement | Acceptance evidence |
| --- | --- | --- |
| S1 | Both hands physically contact `object::main::collision`. | Raw `/vectorview/iCub_fixed/{l,r}_hand` messages. |
| S2 | Arrow geometry is generated inside Gazebo from filtered contact force; no image script adds force visuals. | Blue pixels already exist in raw `/arrow_camera`; capture script contains no overlay call. |
| S3 | The upper object remains stacked on the base during the press. | Static upper object in the release-demo world and visible camera evidence. |
| D1 | One force plot is produced per hand from the same live run. | `/vectorview/...` transport subscriptions started before animation. |
| D2 | Each plot shows original and filtered force in newtons over time, including contact transients. | At least 200 samples, 20 messages, 10 non-zero samples, 10 seconds, and a peak above 1 N per hand. |
| D3 | Camera frames, contacts, and plots come from one simulation execution. | One `capture_comparison.sh` process owns the run and invokes the verifier before layout. |

## Integrity requirements

| ID | Requirement | Acceptance evidence |
| --- | --- | --- |
| I1 | No fake red or blue arrows may be painted in post-processing. | Raw-frame hashes, zero red overlay pixels, and verifier source audit. |
| I2 | A missing contact, arrow, trace, or complete camera view must fail the command instead of producing a success image. | `verify_no_regression.py` exits non-zero on every unmet assertion. |
| I3 | Raw evidence remains available independently of the composed comparison. | `execution_current_raw.png`, `execution_arrows_raw.png`, contact-derived JSON report. |

## Reproduction

```bash
cmake --build build
scripts/capture_comparison.sh
```

Expected terminal verdict:

```text
NO-REGRESSION PASS
```
