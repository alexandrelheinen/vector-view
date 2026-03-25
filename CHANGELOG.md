# Changelog

All notable changes to VectorView & VectorGUI are documented in this file.

---

## [1.0.0] – 2015 (Released 2025)

This is the inaugural stable release of the **VectorView** Gazebo visual plugin and the companion **VectorGUI** desktop application, originally developed in 2015 during an internship at [ISIR](http://www.isir.upmc.fr/) (Institut des Systèmes Intelligents et de Robotique, CNRS / Université Pierre et Marie Curie).

### Context

The work was carried out as part of research on whole-body contact force control for the [iCub](https://icub.iit.it/) humanoid robot. The goal was to make contact forces arising during physical interaction **visible in real time** inside the Gazebo physics simulator, providing researchers with immediate spatial and numerical feedback during experiments.

---

### What's included in v1.0.0

#### VectorView — Gazebo Visual Plugin (`libvectorview.so`)

- Attaches to any `<visual>` element of an iCub model link in a Gazebo world.
- Subscribes automatically to the Gazebo contact sensor topic derived from the link name (convention: `LINK_NAME_contact`).
- Accumulates and averages all contact wrenches reported by the sensor in each message.
- Applies a 3-channel Butterworth low-pass filter (order 3, 1.5 Hz cut-off at 25 Hz sample rate) to the raw X/Y/Z force components before rendering, removing sensor noise.
- Renders a real-time 3-D force vector as a `DynamicLines` object using the **Gazebo Blue** material:
  - A shaft line from the link origin to the scaled force endpoint.
  - Two short wing lines forming an arrowhead at the tip (10° half-angle), correctly oriented along the force direction.
- Applies a noise threshold (`1E-3 N`) so that very small forces do not produce visual clutter.
- Scales the force magnitude to scene units via `FORCE_SCALE = 8E-2 m·N⁻¹`.
- Prints plugin initialisation information (subscribed topic, collision name) to stdout.

#### VectorGUI — Qt4 Desktop Application (`vectorGUI`)

- Connects to any Gazebo contact sensor topic at runtime (full path or short link name accepted).
- Displays live contact information:
  - **Object** – name of the external body in contact.
  - **Position** – 3-D Cartesian coordinates (x, y, z) of the contact point.
  - **Force** – filtered X/Y/Z force components.
- Streams force magnitude data to a real-time **QCustomPlot** graph showing both the raw magnitude and the filtered signal simultaneously.
- Provides a **Spawn** panel where the user can select a model (sphere, cylinder, box, robot, table) from a drop-down menu, enter a target (x, y, z) position, and spawn the object into the running simulation via the Gazebo factory topic.
- Uses a `boost::mutex` to protect shared data between the Gazebo transport callback thread and the Qt timer thread.

#### DSPFilters — Bundled C++ DSP Library

- Local copy of [DSPFilters](https://github.com/vinniefalco/DSPFilters) by Vinnie Falco, providing cascaded digital filters.
- A custom `ForceFilter` wrapper class (`Dsp::ForceFilter`) tailors the library for 3-axis force vector filtering.

#### QCustomPlot — Bundled Qt Plotting Widget

- Local copy of [QCustomPlot](http://www.qcustomplot.com/), included as a single `.cpp`/`.h` pair, enabling the real-time magnitude plot with zero additional Qt dependencies.

#### Build System

- CMake 2.8+ build system with two independent options:
  - `BUILD_VECTORVIEW` (default: `ON`) – builds `libvectorview.so`.
  - `BUILD_VECTORGUI` (default: `ON`) – builds `vectorGUI` executable and pulls in Qt4.
- `DspFilters` is compiled as a shared library used by both targets.

#### Models

- Pre-configured iCub Gazebo model variants with contact sensors already set up:
  - `icub` – base model.
  - `icub_contact` – model with contact sensors.
  - Primitive shapes (`box`, `cylinder`, `sphere`) for spawning during tests.

#### Auxiliary Files

- `robot.world` – ready-to-use Gazebo world file loading the instrumented iCub model.
- `run.sh` – shell script that starts YARP server, Gazebo, two VectorGUI instances (left and right hand), and the ISIR whole-body controller task sequence in separate terminal tabs.

---

### Known limitations at v1.0.0

> The following issues were catalogued as part of the v1.0.0 release process. They are documented in detail in [`docs/issues-report.md`](docs/issues-report.md) and addressed by the roadmap in [`docs/fix-plan.md`](docs/fix-plan.md).

- Uses the **deprecated Gazebo 2–6 C++ math API** (`gazebo::math::Vector3`, `math::Matrix3`, `math::Pose`). Gazebo 7+ replaced this with `ignition::math`.
- Depends on **Qt4**, which reached end-of-life in December 2015.
- CMake minimum version set to 2.8 (released 2009); modern projects require at least 3.5.
- `filter` raw pointer in `VectorView` is allocated in `Load()` but never freed (memory leak).
- `ForceFilter` destructor is declared but not implemented, leaking the inner `Dsp::Filter*`.
- `NOISE_THRESHOLD` is defined independently in two headers with different values (`1E-3` vs `1E-6`).
- No automated tests; no CI/CD pipeline.
- No `LICENSE` file.
- Hard-coded robot name `"iCub"` in `Interface.cpp`.

---

### Dependency versions at time of development (2015)

| Dependency | Version (approx.) |
|---|---|
| Gazebo | 5 / 6 |
| Qt | 4.x |
| YARP | 2.3.x |
| Boost | 1.54+ |
| Protobuf | 2.x |
| CMake | 2.8.x |
| DSPFilters | commit bundled 2015 |
| QCustomPlot | 1.x bundled 2015 |

---

### Contributors

- **Alexandre L. Heinen** — design, implementation, integration (internship at ISIR, 2015).

---

*For a detailed analysis of strengths, identified issues, and the improvement roadmap, see the [`docs/`](docs/) folder.*
