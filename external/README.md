# External dependencies

This directory contains third-party libraries vendored for offline builds. Each
component retains its original license; see the header comments in its sources.

| Directory | Purpose | License |
|-----------|---------|---------|
| `catch2/` | Unit test framework (Catch2 v2.13.10) | Boost Software License 1.0 |
| `dspfilters/` | Butterworth low-pass filter for force smoothing | MIT |
| `qcustomplot/` | Qt plotting widget for Vector GUI (v2.1.1) | GPL v3 or commercial |

Do not reformat files under this directory; `scripts/format.sh` excludes them.
