# v1.0.0 audit and modernization notes

This document consolidates the original v1.0.0 release audit (issues, strengths, and fix
roadmap). The incremental release plan (`v1.0.1` → `v1.1` → `v1.2` → `v2.0`) was
**superseded** by a single **v2.0.0** Noble port that addressed every phase at once.

For the modernization effort estimate and elapsed time, see [`estimate.md`](estimate.md).

---

## What was done well (v1.0.0)

The 2015 internship codebase had solid architectural choices that carried forward into v2.0:

| Aspect | Verdict |
|--------|---------|
| Plugin / GUI decoupling | Clean two-target design |
| Signal processing | Butterworth low-pass filter, well-encapsulated in `ForceFilter` |
| Third-party bundling | Zero-extra-dep build (DSPFilters, QCustomPlot) |
| Arrowhead geometry | Normalised 3-line arrow with ±10° wings |
| Coordinate frame transform | World→local force rotation applied correctly |
| Thread safety (GUI) | RAII mutex between transport and Qt timer threads |
| CMake build options | Conditional headless build (`BUILD_VECTOR_GUI=OFF`) |
| Dual-trace real-time plot | Raw and filtered magnitude shown together |
| In-simulation model spawning | Gazebo factory / EntityFactory integration |
| README and demo script | Step-by-step install and multi-process `run.sh` |

---

## Issues found in v1.0.0 and v2.0 resolution

| # | Severity | Issue | v2.0 status |
|---|----------|-------|-------------|
| 1.1 | Critical | `ForceFilter*` never freed in plugin destructor | Fixed — `std::unique_ptr` |
| 1.2 | Critical | `ForceFilter` destructor not implemented | Fixed — `std::unique_ptr<Dsp::Filter>` |
| 1.3 | High | Raw `DynamicLines*` pointer | N/A — marker-based rendering in Harmonic plugin |
| 2.1 | Critical | Signed/unsigned `n` in force averaging | Fixed — shared `ContactUtils` |
| 2.2 | High | `FindName()` UB on empty names | N/A — replaced by `TopicPath` |
| 2.3 | Medium | `NOISE_THRESHOLD` defined twice | Fixed — `include/vectorview/Constants.h` |
| 2.4 | Medium | Unbounded plot data vectors | Fixed — direct `addData()` calls |
| 3.1 | Critical | Deprecated `gazebo::math::*` API | Fixed — `gz-math` / `Vec3` throughout |
| 3.2 | Critical | Qt4 dependency (EOL) | Fixed — Qt 6.4 |
| 3.3 | High | CMake 2.8 minimum | Fixed — CMake 3.16+, target-based |
| 3.4 | Medium | No C++ standard specified | Fixed — C++17 |
| 3.5 | Low | `FILE(GLOB …)` for sources | Fixed — explicit source lists |
| 4.1 | Medium | `using namespace gazebo` in header | Fixed — `vectorview` namespace |
| 4.2 | Low | Hard-coded robot name `"iCub"` | Fixed — CLI argument + settings panel |
| 4.3 | Low | Commented-out dead code | Removed in rewrite |
| 4.4 | Low | Typo `"collsion"` in log output | Fixed |
| 4.5 | Low | `counter` reset logic in GUI | Fixed |
| 5.1 | High | No `LICENSE` file | Fixed — GPL v3 at repo root |
| 5.2 | High | No automated tests | Fixed — Catch2 unit tests |
| 5.3 | Medium | No CI/CD pipeline | Fixed — GitHub Actions on Noble |
| 5.4 | Medium | No Doxygen / API documentation | Open — not blocking release |
| 5.5 | Low | `run.sh` requires `gnome-terminal` | Fixed — terminal auto-detection |

**Original totals:** 5 critical · 5 high · 6 medium · 6 low. All blocking items resolved in v2.0.

---

## Superseded release schedule

The original incremental plan from the v1.0 audit:

| Milestone | Planned deliverable | Outcome |
|-----------|---------------------|---------|
| `v1.0.1` | Critical bug fixes on Classic stack | Skipped |
| `v1.1.0` | Qt5 + Gazebo 9 compatibility | Skipped |
| `v1.2.0` | Code quality pass on Classic stack | **Skipped — do not release** |
| `v2.0.0` | Tests, CI, licence, Noble port | **Shipped** (tag `v2.0`) |

Releasing `v1.2` on the Classic stack today would be a regression. The current supported
line is **v2.0.0** (Gazebo Harmonic + Qt 6 on Ubuntu 24.04 Noble).
