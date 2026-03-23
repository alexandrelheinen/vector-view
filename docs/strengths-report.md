# What Was Done Well — VectorView & VectorGUI

> **Scope:** This report evaluates the v1.0.0 codebase of VectorView and VectorGUI against software-engineering and robotics-middleware best practices.  
> **Verdict first:** For a 2015 internship project targeting a highly specialised robotics simulation stack, the codebase demonstrates a solid grasp of the problem domain, clean architectural thinking, and several engineering choices that hold up well even a decade later.

---

## 1. Architecture & Separation of Concerns

**VectorView** (the Gazebo visual plugin) and **VectorGUI** (the Qt desktop application) are cleanly decoupled into independent compilation targets. They share no headers apart from the bundled third-party libraries. This means either component can be built, deployed, and studied independently — a discipline that is easy to skip in a time-boxed internship but was respected here.

The three-layer model — *sensor → transport → rendering/GUI* — maps directly onto Gazebo's own design philosophy. The plugin does not reach into simulation internals beyond what the transport layer exposes; the GUI does not know about the render engine. The boundary is clean.

---

## 2. Signal Processing — Noise Filtering Before Rendering

One of the most impactful decisions in the project is the application of a **3-channel Butterworth low-pass filter** to the raw contact force before either rendering the vector or updating the GUI.

Raw Gazebo contact sensor data is notoriously noisy (impulse spikes from the physics integrator, floating-point collision artefacts). Without filtering, the visualised vector would flicker chaotically and be useless for analysis. The choice of:

- Butterworth design (maximally flat magnitude in passband)
- Order 3 (good roll-off, low phase lag for a real-time system)
- 1.5 Hz cut-off at 25 Hz sample rate

…is well-matched to the time scales of whole-body contact events on iCub (~0.1–2 Hz for planned interactions). The signal processing concern is encapsulated in a dedicated `ForceFilter` wrapper, keeping it out of the plugin and GUI logic.

---

## 3. Bundled Third-Party Libraries

Both **DSPFilters** and **QCustomPlot** are bundled directly in `src/` and `include/`. This was a deliberate choice with real engineering merit:

- **Zero extra dependencies** for the consumer; `cmake .. && make` is all that is needed beyond the declared system packages.
- No version drift — the exact revision tested by the author is always used.
- The bundles are self-contained and do not pollute the system library path.

This pattern is common in embedded and robotics projects where the target environment is controlled and reproducibility matters more than keeping up with upstream.

---

## 4. Force Vector Rendering — Arrowhead Geometry

The `UpdateVector()` method does not simply draw a line segment. It draws a proper **arrow** with a two-wing arrowhead:

```cpp
this->forceVector->SetPoint(3, end - ARROW_LENGTH * M_10deg * (end - begin).Normalize());
this->forceVector->SetPoint(5, end - ARROW_LENGTH * M_neg10deg * (end - begin).Normalize());
```

The rotation matrices encode a ±10° deflection from the shaft direction. The arrow is:

- Normalised along the force direction (so arrowhead size does not scale with magnitude).
- Rendered as a `RENDERING_LINE_LIST` — three separate line segments in one call — which is the most GPU-efficient primitive for this use case.

This attention to visual correctness goes beyond what is strictly necessary and improves the usability of the tool for researchers.

---

## 5. Coordinate Frame Handling

The rotation transform in `UpdateVector()`:

```cpp
math::Vector3 end = begin + FORCE_SCALE * (visual->GetWorldPose().rot.RotateVectorReverse(force));
```

correctly converts the force from **world frame** (the frame in which Gazebo reports wrench values) into the **local link frame** (the frame of the visual's coordinate system). Omitting this transform — a common mistake — would cause the vector to drift as the robot moves. The explicit comment referencing the alternative `GetRotation()` call shows the author reasoned carefully about the choice.

---

## 6. Thread Safety in VectorGUI

The `Interface::Update()` callback runs on a Gazebo transport thread, while `Interface::UpdatePlot()` runs on a Qt timer thread. Both access shared `QVector` data. A `boost::mutex::scoped_lock` guards every access:

```cpp
void Interface::Update(ConstContactsPtr &message) {
  boost::mutex::scoped_lock lock(mutex);
  ...
}

void Interface::UpdatePlot() {
  boost::mutex::scoped_lock lock(mutex);
  ...
}
```

RAII-style locking is the correct pattern. A missed lock or a raw `mutex.lock()` without a corresponding `unlock()` in the exception path would cause deadlocks; the scoped lock handles both.

---

## 7. CMake Build Options

The `CMakeLists.txt` exposes two user-facing options:

```cmake
option(BUILD_VECTORGUI  "Enables VectorGUI interface installation" TRUE)
option(BUILD_VECTORVIEW "Either VectorView Visual Plugin is built or not" TRUE)
```

This lets users on headless systems skip the Qt4 dependency entirely by passing `-DBUILD_VECTORGUI=OFF`. It is a small touch, but it shows awareness that not every user has (or wants) a display stack.

The `DspFilters` shared library is conditionally compiled only when at least one of the two main targets is built, avoiding unnecessary work.

---

## 8. Real-Time Plot with Dual Traces

The QCustomPlot graph displays both the **raw** force magnitude and the **filtered** signal simultaneously. This dual-trace design lets the operator evaluate signal quality at a glance — they can see exactly how much noise the filter is removing. Most quick-and-dirty GUI tools would show only one trace; showing both is a conscious observability decision.

---

## 9. Model Spawning During Live Simulation

The `Interface::Spawn()` method publishes a `gazebo::msgs::Factory` message to the `~/factory` topic, which instructs the running Gazebo server to insert a new model at a specified pose without restarting the simulation. This is non-trivial integration work and enables interactive experiments (dropping objects onto the robot's hands while the controller is running).

---

## 10. README Quality

The `README.md` covers:

- What the project does (concise abstract)
- Full dependency list with links
- Step-by-step installation (`git clone`, `mkdir build`, `cmake ..`, `make`)
- Environment variable setup with copy-paste commands
- End-to-end test sequence (three-terminal workflow)
- Shell script usage
- Screenshots

For a 2015 university internship project, this is unusually thorough documentation. The reader can go from zero to running simulation with clearly listed steps.

---

## 11. Product Description (`_products/icub-vector-view.md`)

The project includes a polished product write-up (`_products/icub-vector-view.md`) that explains the research context (ISIR, iCub), the technical approach (DSP filtering, Gazebo transport, Qt GUI), and provides external links. This kind of "why does this exist?" document is rare in student/internship projects and demonstrates communication awareness beyond the code itself.

---

## 12. Shell Script (`run.sh`) for Reproducible Startup

Starting the full stack requires four processes in a specific order with timing dependencies. `run.sh` encapsulates this orchestration: it opens each process in its own terminal tab with `gnome-terminal --tab`, inserts `sleep` delays to respect startup order, and demonstrates both ways to invoke VectorGUI (full topic path vs. short link name). This lowers the barrier to entry for a new developer trying to reproduce the experiment.

---

## Summary Table

| Aspect | Verdict |
|---|---|
| Plugin / GUI decoupling | ✅ Clean two-target design |
| Signal processing | ✅ Correct filter choice, well-encapsulated |
| Third-party bundling | ✅ Zero-extra-dep build |
| Arrowhead geometry | ✅ Correct normalised 3-line arrow |
| Coordinate frame transform | ✅ World→local correctly applied |
| Thread safety | ✅ RAII mutex in both shared-data paths |
| CMake build options | ✅ Conditional headless build supported |
| Dual-trace real-time plot | ✅ Observability above minimum viable |
| In-simulation model spawning | ✅ Non-trivial Gazebo factory integration |
| README / documentation | ✅ Step-by-step, links, screenshots |
| Product write-up | ✅ Context, motivation, resources |
| Startup script | ✅ Reproducible multi-process orchestration |
