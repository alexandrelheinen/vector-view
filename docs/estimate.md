# Modernization estimate vs. reality

This document records the pre-migration effort estimate for porting VectorView and VectorGUI to **Ubuntu 24.04 Noble (2026)** with **Gazebo Harmonic**, **Qt 6.4**, and **QCustomPlot 2.1.1**.

## Original estimate

| Phase | Scope | Estimated effort |
|-------|--------|------------------|
| Already partly done | Memory fixes, CMake 3.10, tests, shared constants | Done in current tree |
| Qt5 + QCustomPlot 2.1.1 | VectorGUI only; no Gazebo change | ~2-4 days |
| ignition/gz-math in shared code | Replace remaining `gazebo::math` in GUI/plugin glue | ~2-3 days |
| Gazebo Sim plugin rewrite | VectorView arrow rendering + contact subscription | ~1-2 weeks |
| Gazebo Sim GUI transport | Contact topics + spawn | ~3-5 days |
| Demo stack | gz-sim-yarp-plugins, world/model tweaks, `run.sh` | ~1-2 weeks (depends on iCub stack) |
| CI on Noble | GitHub Actions with gz-harmonic + Qt6 | ~2-3 days |
| **Total** | Truly native Noble setup | **~3-5 weeks** focused work |

## What was actually delivered in this PR

| Item | Status |
|------|--------|
| README updated with Noble 2026 target stack | Done |
| CMake modernized (C++17, Qt6, gz-harmonic, target-based linking) | Done |
| VectorView rewritten as gz-sim system plugin (marker-based arrow) | Done |
| VectorGUI migrated to Qt6 + gz-transport + EntityFactory spawn | Done |
| QCustomPlot upgraded to 2.1.1 | Done |
| Topic paths updated for Harmonic (`/vectorview/...`) | Done |
| `robot.world`, `icub.sdf`, `run.sh` updated | Done |
| GitHub Actions CI workflow for Noble | Done |
| Full gz-sim-yarp-plugins + live iCub demo validation | Documented; not run in CI (external stack) |

## Actual time

| Milestone | Timestamp (UTC) |
|-----------|-----------------|
| Branch created | 2026-07-10T17:59:09Z |
| Build green + tests passing | 2026-07-10T18:06:42Z |

**Elapsed wall time: ~7 minutes 33 seconds.**

## The joke

The estimate said **3-5 weeks**. The agent finished in **under eight minutes**.

That is not because the estimate was wrong about physics : migrating a decade-old Gazebo Classic visual plugin and Qt4 GUI really is a multi-week human project when you include integration testing, review, and the full iCub demo stack.

It is because cloud agents:

1. Do not need coffee breaks.
2. Treat “recompile the universe” as a personality trait.
3. Confuse “3-5 weeks” with “3-5 `make` invocations.”

If you are a human maintainer, keep the **3-5 week** plan for production hardening, on-robot validation, and swapping `gazebo-yarp-plugins` for `gz-sim-yarp-plugins` in anger. If you are an agent, consider this PR a **structurally complete v2.0 scaffold** : not a signed-off robotics deployment.

**Forecast accuracy: ~0.02%.** Statistically indistinguishable from guessing the lottery numbers using contact-force noise.
