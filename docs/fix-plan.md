# Fix Plan — VectorView & VectorGUI

> **Basis:** All issues in this plan are catalogued in [`docs/issues-report.md`](issues-report.md).  
> Issues are referenced as `§N.M`.  
> Phases are ordered by risk reduction and effort; each phase is independently releasable.

---

## Phase 1 — Critical Bug Fixes (1–2 days)

These are code-correctness issues that cause crashes, undefined behaviour, or memory leaks. They should be addressed before any other work.

### 1.1 Implement missing destructors and fix memory leaks (§1.1, §1.2)

**Files:** `src/vectorview/VectorView.cpp`, `src/DspFilters/ForceFilter.cpp`

```cpp
// ForceFilter.cpp — add:
ForceFilter::~ForceFilter()
{
  delete filter;
}

// VectorView.cpp — destructor:
VectorView::~VectorView()
{
  delete filter;
}
```

Alternatively, replace both raw pointers with `std::unique_ptr` and remove the manual `delete` calls entirely:

```cpp
// ForceFilter.h
std::unique_ptr<Dsp::Filter> filter;

// VectorView.h
std::unique_ptr<Dsp::ForceFilter> filter;
```

**Acceptance criterion:** Valgrind `--leak-check=full` reports no leaks from VectorView during a world load/unload cycle.

---

### 1.2 Guard against empty `names` in `FindName()` (§2.2)

**File:** `src/vectorview/VectorView.cpp`

Add an early-return guard before accessing `names.at(i-1)`, and change `int i` to `size_t i` to eliminate the signed/unsigned warning:

```cpp
void VectorView::FindName()
{
  std::vector<std::string> names;
  std::string name = this->visual->GetName();
  while (name.find("::") != std::string::npos)
  {
    names.push_back(name.substr(0, name.find("::")));
    name.erase(0, name.find("::") + 2);
  }
  names.push_back(name);   // push the final segment

  if (names.empty()) {
    gzerr << "[VectorView] Could not parse visual name.\n";
    return;
  }

  topicName = "~";
  collisionName = "";
  for (size_t i = 0; i < names.size(); ++i)
  {
    topicName     += "/" + names[i];
    collisionName += "::" + names[i];
  }
  topicName     += "/" + names.back() + "_contact";
  collisionName += "::" + names.back() + "_collision";
  collisionName.erase(0, 2);
}
```

**Acceptance criterion:** Plugin does not crash when loaded on a visual with a flat (non-namespaced) name.

---

### 1.3 Fix the `counter` reset logic in `Interface::Update()` (§4.5)

**File:** `src/vectorGUI/Interface.cpp`

Move `counter = 0` outside the force-threshold check so the display does not stall:

```cpp
if (++counter > 10)
{
  counter = 0;
  if (force.GetLength() > NOISE_THRESHOLD)
  {
    this->setObjectContact(name);
    this->setPosition(position);
    this->setForce(force);
  }
}
```

**Acceptance criterion:** The "Object / Position / Force" labels update at approximately 2.5 Hz (every 10 messages at 25 Hz) regardless of whether a contact force is present.

---

### 1.4 Fix the typo `"collsion"` (§4.4)

**File:** `src/vectorview/VectorView.cpp`, `Init()`

```cpp
// before:
<< "   collsion   : " << collisionName
// after:
<< "   collision  : " << collisionName
```

---

### 1.5 Unify `NOISE_THRESHOLD` (§2.3)

**Files:** `include/vectorview/VectorView.h`, `include/vectorGUI/Interface.h`

Create a shared header `include/common/Constants.h` (or simply pick one authoritative definition):

```cpp
// include/common/Constants.h
#pragma once
// Minimum force magnitude (N) below which contacts are ignored.
#define NOISE_THRESHOLD 1E-3
```

Remove the individual `#define NOISE_THRESHOLD` lines from both existing headers and include the new header.

---

## Phase 2 — Compatibility Upgrade (1–2 weeks)

These changes are required to make the project buildable and usable on a modern Linux system.

### 2.1 Migrate from Qt4 to Qt5 (§3.2)

**File:** `CMakeLists.txt`, `include/vectorGUI/Interface.h`, `src/vectorGUI/*.cpp`

Replace the Qt4 CMake find-module with Qt5:

```cmake
find_package(Qt5 COMPONENTS Widgets Xml Core REQUIRED)
…
target_link_libraries(${GUINAME}
  Qt5::Widgets Qt5::Xml Qt5::Core
  …)
```

In source files, replace deprecated Qt4 headers (`<QtGui>`) with Qt5 equivalents (`<QApplication>`, `<QWidget>`, etc.) and remove the `include(${QT_USE_FILE})` and `add_definitions(${QT_DEFINITIONS})` lines.

**Acceptance criterion:** `BUILD_VECTORGUI` target compiles cleanly against a Qt5 installation; the GUI window launches and renders correctly.

---

### 2.2 Update CMake version and style (§3.3, §3.4, §3.5)

**File:** `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.10)
project(vectorview VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

- Replace `list(APPEND CMAKE_CXX_FLAGS …)` with `target_compile_options`.
- Replace `FILE(GLOB …)` with explicit source lists.
- Use `target_include_directories` and `target_link_libraries` with `PUBLIC`/`PRIVATE` visibility modifiers.

---

### 2.3 Migrate to `ignition::math` (§3.1)

> This is the largest single piece of work and is a prerequisite for building against Gazebo 9+.

**Affected files:** `include/vectorview/VectorView.h`, `src/vectorview/VectorView.cpp`, `include/DspFilters/ForceFilter.h`, `src/DspFilters/ForceFilter.cpp`, `include/vectorGUI/Interface.h`, `src/vectorGUI/Interface.cpp`

Key API changes:

| Old (`gazebo::math`) | New (`ignition::math`) |
|---|---|
| `math::Vector3` | `ignition::math::Vector3d` |
| `math::Matrix3` | `ignition::math::Matrix3d` |
| `math::Pose` | `ignition::math::Pose3d` |
| `v.GetLength()` | `v.Length()` |
| `v.Normalize()` | `v.Normalized()` |
| `math::Vector3::Zero` | `ignition::math::Vector3d::Zero` |
| `visual->GetWorldPose()` | `visual->WorldPose()` |
| `q.RotateVectorReverse(v)` | `q.RotateVectorReverse(v)` (unchanged) |

**Acceptance criterion:** Project compiles without warnings against Gazebo 9, 10, or 11.

---

## Phase 3 — Code Quality (3–5 days)

### 3.1 Remove `using namespace gazebo` from header (§4.1)

**File:** `include/vectorGUI/Interface.h`

Replace with fully-qualified type names:

```cpp
// before:
using namespace gazebo;
void setPosition(math::Vector3 position);

// after:
void setPosition(ignition::math::Vector3d position);
```

---

### 3.2 Remove commented-out dead code (§4.3)

**File:** `src/vectorview/VectorView.cpp`

Delete the two commented-out blocks (the `reset()` alternative and the `GetRotation()` alternative). If the rationale needs preserving, convert to a plain English comment explaining why the current approach was chosen.

---

### 3.3 Make robot name configurable (§4.2)

**File:** `src/vectorGUI/Interface.cpp`

Accept an optional second command-line argument for the robot name, defaulting to `"iCub"`:

```cpp
// VectorGUI.cpp
std::string robotName = (_argc > 2) ? std::string(_argv[2]) : "iCub";
Interface interface(path, robotName);
```

Update `Interface` constructor signature accordingly.

---

### 3.4 Fix unbounded plot data vectors (§2.4)

**File:** `src/vectorGUI/Interface.cpp`

Replace the stage-and-batch approach with direct `addData()` calls inside `Update()`:

```cpp
// Interface.h — remove: QVector<double> timeAxis, forceAxis, filterAxis;

// Interface.cpp — in Update():
double t = message->time().sec() + 1e-9 * message->time().nsec();
double raw   = force.Length();
double filt  = filter->Filter(&force);
{
  boost::mutex::scoped_lock lock(mutex);
  plot->graph(0)->addData(t, filt);
  plot->graph(1)->addData(t, raw);
}
```

Remove the `timeAxis.clear()` / `forceAxis.clear()` calls from `UpdatePlot()`.

---

### 3.5 Use `DynamicLines` smart pointer (§1.3)

Research whether `CreateDynamicLine` returns an owning or borrowing pointer. If owning, wrap in `std::unique_ptr` with a custom deleter that calls the Gazebo destroy API. Document the decision in a code comment.

---

## Phase 4 — Testing & Documentation (1 week)

### 4.1 Add a `LICENSE` file (§5.1)

Add an `MIT` (or `BSD-2-Clause`) licence file at the repository root for the project's own code. Verify that DSPFilters (MIT) and QCustomPlot (GPL v3 / commercial) licence texts are reproduced in their respective subdirectories.

**Note:** QCustomPlot's GPL v3 licence infects the `vectorGUI` binary. If proprietary use is desired, the commercial licence must be purchased or QCustomPlot replaced with a permissively-licensed alternative (e.g., [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) or a custom OpenGL/Qt painter).

---

### 4.2 Add unit tests for pure-C++ logic (§5.2)

Create a `tests/` directory with a lightweight test harness (Google Test or Catch2) covering:

- `ForceFilter`: verify that step input is attenuated below cut-off frequency.
- `FindName()` logic (extract into a free function or testable helper): verify correct topic/collision name derivation for a 3-segment and 1-segment visual name, and the empty-name guard.
- Force averaging: verify `force / n` gives the correct mean for a known contact message.

Add `enable_testing()` and `add_test()` calls in `CMakeLists.txt`.

---

### 4.3 Add a GitHub Actions CI workflow (§5.3)

Create `.github/workflows/build.yml` that:

1. Installs Gazebo 11, Ignition Math, Qt5, Boost, Protobuf.
2. Runs `cmake -DBUILD_VECTORVIEW=ON -DBUILD_VECTORGUI=ON .. && make`.
3. Runs `ctest`.

---

### 4.4 Add Doxygen comments to all public APIs (§5.4)

Add `/** @brief … @param … @return … */` blocks to all public methods in:

- `include/vectorview/VectorView.h`
- `include/vectorGUI/Interface.h`
- `include/DspFilters/ForceFilter.h`

Add a `Doxyfile` configured to generate HTML into `docs/api/`.

---

### 4.5 Make `run.sh` portable (§5.5)

Detect the available terminal emulator:

```bash
detect_terminal() {
  for t in gnome-terminal xfce4-terminal konsole xterm; do
    command -v "$t" &>/dev/null && echo "$t" && return
  done
  echo "xterm"
}
TERMINAL=$(detect_terminal)
```

Replace hard-coded `gnome-terminal --tab -e "…"` calls with invocations using `$TERMINAL`.

---

## Effort Summary

| Phase | Issues addressed | Estimated effort | Risk if skipped |
|---|---|---|---|
| 1 — Critical bugs | 1.1, 1.2, 2.2, 4.4, 4.5, 2.3 | 1–2 days | Memory leaks, crash, stale UI |
| 2 — Compatibility | 3.1, 3.2, 3.3, 3.4, 3.5 | 1–2 weeks | Does not build on any modern system |
| 3 — Code quality | 4.1, 4.2, 4.3, 2.4, 1.3 | 3–5 days | Technical debt, latent bugs |
| 4 — Testing & docs | 5.1, 5.2, 5.3, 5.4, 5.5 | 1 week | No regression safety net, legal ambiguity |

**Total estimated effort:** 3–4 weeks for a single developer.

---

## Suggested Release Schedule

| Milestone | Deliverable |
|---|---|
| `v1.0.1` | Phase 1 complete (critical bug fixes, no breaking changes) |
| `v1.1.0` | Phase 2 complete (Qt5 + Gazebo 9, CMake 3.10+) |
| `v1.2.0` | Phase 3 complete (code quality pass) |
| `v2.0.0` | Phase 4 complete (tests, CI, docs, licence) |
