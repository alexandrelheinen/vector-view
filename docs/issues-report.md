# Issues Report — VectorView & VectorGUI (v1.0.0)

> **Purpose:** A complete technical audit of all defects, anti-patterns, and risks identified in the v1.0.0 codebase.  
> Issues are grouped by category and assigned a severity: 🔴 **Critical** | 🟠 **High** | 🟡 **Medium** | 🔵 **Low / Style**.

---

## Category 1 — Memory Management

### 1.1 🔴 Memory leak — `ForceFilter` in `VectorView::Load()`

**File:** `src/vectorview/VectorView.cpp`, `VectorView::Load()`

```cpp
void VectorView::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  this->visual = _parent;
  filter = new Dsp::ForceFilter();   // ← allocated here …
}
```

`VectorView::~VectorView()` is an empty body. The `Dsp::ForceFilter*` object is never freed. In a long-running simulation this leaks memory every time a world is reloaded and the plugin is re-instantiated.

**Fix:** `delete filter;` in the destructor (or use `std::unique_ptr<Dsp::ForceFilter>`).

---

### 1.2 🔴 `ForceFilter` destructor declared but never defined

**File:** `include/DspFilters/ForceFilter.h`

```cpp
class ForceFilter {
public:
  ForceFilter();
  ~ForceFilter();           // declared …
  …
private:
  Dsp::Filter* filter;      // raw pointer
};
```

`ForceFilter.cpp` contains only the constructor and `Filter()`. The destructor is declared but has no implementation. Linking succeeds (the compiler generates a no-op thunk) but the inner `Dsp::Filter*` is therefore **never deleted**, creating a second memory leak every time a `ForceFilter` is constructed and destroyed.

**Fix:** Implement `ForceFilter::~ForceFilter() { delete filter; }`.

---

### 1.3 🟠 Raw pointer for `DynamicLines` (`LinePtr`)

**File:** `include/vectorview/VectorView.h`

```cpp
typedef rendering::DynamicLines* LinePtr;
LinePtr forceVector;
```

The typedef comment acknowledges the original intent to use `boost::shared_ptr` (the commented-out alternative). A raw pointer provides no automatic lifetime management. If `Init()` is called more than once (e.g., Gazebo world reset), the previous `DynamicLines` object would be leaked and a dangling pointer would remain.

**Fix:** Check Gazebo's ownership semantics; if the visual owns the line, document it explicitly. Otherwise use `std::unique_ptr` or reset before re-creating.

---

## Category 2 — Logic Errors

### 2.1 🔴 Division by `n` after loop — wrong value used

**File:** `src/vectorview/VectorView.cpp`, `VectorViewUpdate()`

```cpp
unsigned int n, m;
for(n = 0; n < message->contact_size(); ++n)
{
  for (m = 0; m < message->contact(n).wrench_size(); ++m)
  { … }
}

if(message->contact_size())
{
  force = force / n;   // ← n == message->contact_size() here, which is correct
}
```

At this point `n == message->contact_size()`, which is the correct divisor for the mean. However, **in `Interface::Update()`** the same pattern is used with a `signed int`:

```cpp
int n, m;
for(n = 0; n < message->contact_size(); ++n) { … }
force = force / n;
```

Here `n` is a **signed `int`** and Gazebo's `contact_size()` returns a `uint32`. The implicit narrowing is benign when contact counts are small, but the inconsistency between `VectorView` (unsigned) and `Interface` (signed) is a latent bug: if `contact_size()` exceeds `INT_MAX` in a degenerate scenario the signed `n` would wrap and produce a garbage divisor.

**Fix:** Use `int n = message->contact_size(); force = force / n;` after the loop, making the intent explicit.

---

### 2.2 🟠 Undefined behaviour risk in `FindName()` — `i` used after loop

**File:** `src/vectorview/VectorView.cpp`, `FindName()`

```cpp
int i;
for(i = 0; i < names.size(); ++i)
{
  topicName     += "/" + names.at(i);
  collisionName += "::" + names.at(i);
}
topicName     += "/" + names.at(i-1) + "_contact";   // ← i-1 after loop
collisionName += "::" + names.at(i-1) + "_collision";
```

If `names` is **empty** (e.g., the visual name contains no `::` separator), `i == 0` after the loop, and `names.at(i-1)` is `names.at(-1)` — an out-of-range access on a signed-to-unsigned wrap, producing **undefined behaviour** (likely a crash or garbage string).

The variable `i` is also declared as signed `int` while `names.size()` is `size_t` (unsigned), causing a signed/unsigned comparison warning on every compliant compiler.

**Fix:** Guard with `if (names.empty()) { /* error or fallback */ return; }` before the loop; use `size_t` or `int` consistently.

---

### 2.3 🟡 `NOISE_THRESHOLD` defined twice with different values

**File:** `include/vectorview/VectorView.h` and `include/vectorGUI/Interface.h`

```cpp
// VectorView.h
#define NOISE_THRESHOLD 1E-3

// Interface.h
#define NOISE_THRESHOLD 1E-6
```

The two translation units use different thresholds. When both headers are included in the same TU the later definition silently overrides the first (no `#ifdef` guard). The visual plugin and the GUI therefore apply different noise gates, producing visually inconsistent behaviour — the GUI can show forces that the plugin does not render (or vice versa).

**Fix:** Centralise `NOISE_THRESHOLD` in a single shared header or make it a runtime parameter.

---

### 2.4 🟡 Plot data vectors are unbounded in memory

**File:** `src/vectorGUI/Interface.cpp`, `Update()` / `UpdatePlot()`

```cpp
// Update() — runs at every contact message (~25 Hz):
this->timeAxis.push_back(…);
this->forceAxis.push_back(…);
this->filterAxis.push_back(…);

// UpdatePlot() — runs at RATE Hz:
plot->graph(0)->addData(timeAxis, forceAxis);
…
timeAxis.clear();  forceAxis.clear();  filterAxis.clear();
```

There is a `TODO` comment in the source acknowledging this. If `UpdatePlot()` falls behind `Update()` (or if the Qt timer fires less frequently than data arrives), the `QVector`s grow without bound. In a two-hour experiment at 25 Hz the vectors would hold 180,000 elements before each clear — a 1.4 MB allocation spike per refresh cycle.

**Fix:** Use `plot->graph(n)->addData(time, value)` as the TODO suggests, accumulating directly into QCustomPlot's internal structure instead of staging in unbounded vectors.

---

## Category 3 — API and Compatibility

### 3.1 🔴 Gazebo deprecated math API (`gazebo::math::*`)

**Files:** `VectorView.cpp`, `Interface.cpp`, `ForceFilter.h/.cpp`

The entire codebase uses the `gazebo::math::` namespace (`Vector3`, `Matrix3`, `Pose`, `Quaternion`). This namespace was **deprecated in Gazebo 7 (2016)** and **removed in Gazebo 9 (2018)**. The library was replaced by `ignition::math`.

The project will **not compile against any Gazebo version from 2018 onward** without modification.

```cpp
// deprecated — Gazebo ≥ 9 will not compile this:
math::Vector3 force = math::Vector3::Zero;
math::Vector3 begin = math::Vector3::Zero;
visual->GetWorldPose().rot.RotateVectorReverse(force);
```

**Fix:** Replace all `gazebo::math::` types with `ignition::math::` equivalents (mostly mechanical find-and-replace plus method name changes, e.g., `GetLength()` → `Length()`).

---

### 3.2 🔴 Qt4 dependency (end-of-life December 2015)

**File:** `CMakeLists.txt`

```cmake
find_package(Qt4 REQUIRED QtGui QtXml QtCore)
include(${QT_USE_FILE})
```

Qt4 reached end-of-life at the same time this project was written. Qt5 is API-compatible for most widget code, but the CMake integration differs (`find_package(Qt5 ...)` vs `find_package(Qt4 ...)`). No modern Linux distribution ships Qt4 in its default package repositories.

**Fix:** Migrate to `find_package(Qt5 COMPONENTS Widgets Xml Core REQUIRED)` and update the corresponding `target_link_libraries` calls.

---

### 3.3 🟠 CMake minimum version 2.8 (released 2009)

**File:** `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
```

CMake 2.8 dates from 2009. Features used in modern CMake (target-scoped includes, `target_compile_options`, generator expressions, `CMAKE_CXX_STANDARD`) are not available at this version. Several commands used in the file (`list(APPEND CMAKE_CXX_FLAGS …)`, `FILE(GLOB …)`) are considered deprecated practice in modern CMake.

**Fix:** Set `cmake_minimum_required(VERSION 3.10)` and adopt target-based CMake idioms.

---

### 3.4 🟡 No C++ standard specified in CMakeLists.txt

**File:** `CMakeLists.txt`

No `set(CMAKE_CXX_STANDARD 11)` or `target_compile_features` directive is present. The C++ standard used is whatever the compiler's default is, which varies by compiler version. Given the use of range-based loops and `nullptr` in Gazebo headers, C++11 is the minimum required.

**Fix:** Add `set(CMAKE_CXX_STANDARD 11)` and `set(CMAKE_CXX_STANDARD_REQUIRED ON)`.

---

### 3.5 🔵 `FILE(GLOB …)` for source discovery

**File:** `CMakeLists.txt`

```cmake
FILE(GLOB filter_source ${FILTER_SOURCE}/*.cpp)
FILE(GLOB filter_header ${FILTER_INCLUDE}/*.h)
```

CMake documentation explicitly recommends against `FILE(GLOB …)` because CMake does not re-run when a new source file is added to the directory, causing stale build systems. New files are silently ignored until the developer manually re-runs CMake.

**Fix:** List sources explicitly.

---

## Category 4 — Code Quality

### 4.1 🟡 `using namespace gazebo` in a header file

**File:** `include/vectorGUI/Interface.h`

```cpp
using namespace gazebo;

class Interface : public QWidget { … };
```

`using namespace` in a header forces every translation unit that includes `Interface.h` to pull in the entire `gazebo` namespace, which risks name collisions with system or Qt symbols. This is universally considered bad practice in C++ (see C++ Core Guidelines SF.7).

**Fix:** Remove the `using namespace gazebo;` from the header; use fully-qualified names (`gazebo::math::Vector3`, etc.) or forward declarations.

---

### 4.2 🔵 Hard-coded robot name `"iCub"`

**File:** `src/vectorGUI/Interface.cpp`

```cpp
std::string robotName = "iCub";
if (message->contact(n).wrench(m).body_1_name().find(robotName) != std::string::npos)
```

The GUI will misidentify contact sides for any robot model whose name does not contain the string `"iCub"`. This is undocumented and breaks the otherwise generic design of the tool.

**Fix:** Accept the robot name as a constructor parameter or command-line argument, with `"iCub"` as the default.

---

### 4.3 🔵 Commented-out dead code

**File:** `src/vectorview/VectorView.cpp`

```cpp
//this->forceVector.reset(this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP));
…
//visual->GetRotation().RotateVectorReverse(force)));
```

Two multi-line sections of code are commented out. One shows an alternative `LinePtr` type (smart pointer), the other an alternative rotation call. These should either be removed or converted to explanatory comments.

---

### 4.4 🔵 Typo in console output

**File:** `src/vectorview/VectorView.cpp`, `Init()`

```cpp
<< "   collsion   : " << collisionName << std::endl
```

`"collsion"` should be `"collision"`. Minor, but it affects log readability.

---

### 4.5 🔵 Inconsistent `counter` reset logic in `Interface::Update()`

**File:** `src/vectorGUI/Interface.cpp`

```cpp
if(++counter > 10)
{
  if(force.GetLength() > NOISE_THRESHOLD)
  {
    this->setObjectContact(name);
    …
    counter = 0;    // reset only if force > threshold
  }
}
```

`counter` is reset to zero only when a contact with sufficient force is detected. If no qualifying contact arrives, `counter` keeps incrementing past 10 indefinitely. The labels are never updated until a contact exceeds the threshold. This creates a silent "stale display" scenario that is not documented.

**Fix:** Reset `counter` unconditionally inside the `> 10` block, regardless of force magnitude.

---

## Category 5 — Project / Repository Hygiene

### 5.1 🟠 No `LICENSE` file

The repository has no `LICENSE` or `COPYING` file. Bundled third-party code (DSPFilters — MIT; QCustomPlot — GPL/Commercial dual-licence) has specific redistribution requirements. Without an explicit project licence, the legal status of the code is ambiguous for any third party wishing to use or contribute.

**Fix:** Add an `MIT` (or `BSD-2`) licence for the project's own code, and ensure bundled libraries' licences are reproduced.

---

### 5.2 🟠 No automated tests

The project has no unit tests, integration tests, or test targets in CMake. The only way to verify correctness is to run the full Gazebo simulation stack — a heavyweight procedure that requires hardware or a configured VM.

At minimum, the `ForceFilter` wrapper and the `FindName()` logic could be tested with a standalone Google Test or Catch2 harness without any Gazebo dependency.

**Fix:** Add a `tests/` directory with CMake test targets for pure-C++ logic.

---

### 5.3 🟡 No CI/CD pipeline

There is no `.github/workflows/` or similar CI configuration. A broken commit that fails to compile can silently sit in the repository indefinitely.

**Fix:** Add a GitHub Actions workflow that at minimum attempts `cmake .. && make` with the required dependencies installed.

---

### 5.4 🟡 No Doxygen / API documentation

Neither the plugin nor the GUI classes have Doxygen-style doc comments on public methods. A new contributor has to read the implementation to understand what `FindName()`, `UpdateVector()`, or `Spawn()` do.

**Fix:** Add `/** @brief … */` comments to all public API methods; add a `Doxyfile` to generate HTML docs.

---

### 5.5 🔵 `run.sh` uses `gnome-terminal` — not portable

**File:** `run.sh`

`gnome-terminal` is specific to GNOME desktop environments. The script silently does nothing on KDE, XFCE, LXDE, or any headless environment.

**Fix:** Use `xterm` as a portable fallback or detect the available terminal emulator at runtime.

---

## Summary Table

| # | Severity | Category | Issue |
|---|---|---|---|
| 1.1 | 🔴 Critical | Memory | `ForceFilter*` never freed in `VectorView` destructor |
| 1.2 | 🔴 Critical | Memory | `ForceFilter::~ForceFilter()` not implemented; inner `Dsp::Filter*` leaked |
| 1.3 | 🟠 High | Memory | Raw `DynamicLines*` pointer — no ownership semantics |
| 2.1 | 🔴 Critical | Logic | Signed/unsigned `n` inconsistency in force averaging |
| 2.2 | 🟠 High | Logic | `names.at(i-1)` UB when `names` is empty in `FindName()` |
| 2.3 | 🟡 Medium | Logic | `NOISE_THRESHOLD` defined twice with different values |
| 2.4 | 🟡 Medium | Logic | Unbounded plot data vectors can cause memory spikes |
| 3.1 | 🔴 Critical | Compat | Deprecated `gazebo::math::*` API — won't compile on Gazebo ≥ 9 |
| 3.2 | 🔴 Critical | Compat | Qt4 dependency — EOL; no modern distro ships it |
| 3.3 | 🟠 High | Compat | CMake minimum 2.8 — too old; blocks modern CMake features |
| 3.4 | 🟡 Medium | Compat | No C++ standard specified in CMakeLists.txt |
| 3.5 | 🔵 Low | Build | `FILE(GLOB …)` for source discovery |
| 4.1 | 🟡 Medium | Quality | `using namespace gazebo` in header |
| 4.2 | 🔵 Low | Quality | Hard-coded robot name `"iCub"` |
| 4.3 | 🔵 Low | Quality | Commented-out dead code |
| 4.4 | 🔵 Low | Quality | Typo `"collsion"` in console output |
| 4.5 | 🔵 Low | Quality | `counter` reset logic in GUI is undocumented and confusing |
| 5.1 | 🟠 High | Hygiene | No `LICENSE` file |
| 5.2 | 🟠 High | Hygiene | No automated tests |
| 5.3 | 🟡 Medium | Hygiene | No CI/CD pipeline |
| 5.4 | 🟡 Medium | Hygiene | No Doxygen / API documentation |
| 5.5 | 🔵 Low | Hygiene | `run.sh` requires `gnome-terminal` |

**Total:** 5 Critical · 5 High · 6 Medium · 6 Low
