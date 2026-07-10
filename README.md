# VectorView

Visualize iCub contact forces in Gazebo. This repository contains two programs that work together:

| Component | Binary | Role |
|-----------|--------|------|
| **VectorView** | `libvectorview.so` | Gazebo visual plugin that draws a force arrow on a link |
| **VectorGUI** | `vectorGUI` | Qt desktop app that displays contact data, plots force magnitude, and spawns models |

Both read from Gazebo transport contact topics. The plugin renders in the 3D view; the GUI provides a separate analysis window.

Bundled assets:

- `models/` contains the instrumented iCub model and simple spawn primitives
- `worlds/robot.world` is the demo Gazebo world

## Contact sensor naming

Each contact sensor must be named after its link:

```text
LINK_NAME_contact
```

Example: link `l_hand` uses sensor `l_hand_contact`.

The plugin is attached to the link visual (for example `l_hand_visual`). Topic names are resolved from the link name, not the visual name. See [Topic paths](#topic-paths) below.

## Repository layout

```text
include/vectorview/     Public headers
src/plugin/             Gazebo visual plugin
src/gui/                Qt application
src/filters/            Butterworth force filter wrapper
src/common/             Shared logic (ContactUtils, TopicPath)
third_party/            Vendored DSPFilters, QCustomPlot, Catch2
models/                 Gazebo models
worlds/                 Gazebo world files
scripts/                Demo launcher and format helper
tests/                  Unit tests (no Gazebo required)
.env.example            Environment variable template
```

## Requirements

### Build the plugin and GUI

These targets target a **legacy Gazebo Classic + Qt4** stack:

| Dependency | Notes |
|------------|-------|
| CMake 3.10+ | Build system |
| C++11 compiler | GCC or Clang |
| pkg-config | Dependency discovery |
| Gazebo Classic | `pkg-config --modversion gazebo` |
| OGRE 1.9 | Rendering |
| Protobuf, Boost | Pulled in by Gazebo |
| Qt4 | QtGui, QtXml, QtCore |

On **Ubuntu 20.04**, a typical install looks like:

```bash
sudo apt update
sudo apt install cmake g++ pkg-config \
  libgazebo-dev libogre-1.9-dev libprotobuf-dev \
  libboost-system-dev libboost-filesystem-dev \
  libqt4-dev libqt4-opengl-dev
```

Package names vary by distribution and Gazebo version. Adjust as needed.

On **Ubuntu 22.04 and later**, Qt4 and Gazebo Classic are generally not available from the default repositories. You can still [build and run the unit tests](#unit-tests) without them.

### Run the full iCub demo

The original internship workflow also depends on external robotics software:

- [YARP](https://www.yarp.it/) and the iCub software stack
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [ocra-core](https://github.com/ocra-recipes/ocra-core)
- [codyco-superbuild](https://github.com/alexandrelheinen/codyco-superbuild) (provides `ISIRWholeBodyController`)

There is no single `apt install` for this layer. Follow the installation guides linked above.

### Bundled third-party code

Sources for [DSPFilters](https://github.com/vinniefalco/DSPFilters) and [QCustomPlot](http://www.qcustomplot.com/) are vendored under `third_party/`. You do not need to install them separately.

## Quick start

### Clone and build

```bash
git clone https://github.com/alexandrelheinen/vector-view.git
cd vector-view
mkdir build && cd build
cmake ..
make
```

CMake options (all enabled by default):

| Option | Default | Purpose |
|--------|---------|---------|
| `BUILD_VECTORVIEW` | ON | Build the Gazebo plugin |
| `BUILD_VECTORGUI` | ON | Build the Qt application |
| `BUILD_TESTS` | ON | Build unit tests |

If Gazebo or Qt4 is not found, CMake disables the plugin and GUI targets automatically and prints a warning. Tests still build.

Install to system paths (optional):

```bash
sudo make install
```

### Unit tests

Tests cover pure C++ logic and do not require Gazebo or Qt:

```bash
mkdir -p build && cd build
cmake ..
make
ctest --output-on-failure
```

Force a tests-only configure:

```bash
cmake .. -DBUILD_VECTORVIEW=OFF -DBUILD_VECTORGUI=OFF -DBUILD_TESTS=ON
```

## Configuration

Gazebo must locate the built plugin (`.so`) and bundled models. Set this up once with a local environment file:

```bash
cp .env.example .env
```

Edit `.env` if your paths differ from the defaults. The file is loaded by `scripts/run.sh`. Shells do not read `.env` on their own; either source it manually or use the run script.

| Variable | Purpose |
|----------|---------|
| `VECTOR_VIEW` | Repository root (defaults to the directory above `scripts/`) |
| `GAZEBO_PLUGIN_PATH` | Directory containing `libvectorview.so` |
| `GAZEBO_MODEL_PATH` | Directory containing bundled `models/` |
| `CODYCO_SUPERBUILD_ROOT` | Optional. Enables the ISIR controller step in `run.sh` |

Equivalent manual exports:

```bash
export VECTOR_VIEW=/path/to/vector-view
export PATH="$VECTOR_VIEW/build:$PATH"
export GAZEBO_PLUGIN_PATH="$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH:-}"
```

## Running the demo

### Option 1: run script

```bash
chmod +x scripts/run.sh
./scripts/run.sh
```

The script:

1. Sources `.env` when present
2. Exports Gazebo paths
3. Opens YARP, Gazebo, two VectorGUI instances, and (if configured) the ISIR controller in separate `gnome-terminal` tabs

If `CODYCO_SUPERBUILD_ROOT` is unset, the controller step is skipped.

### Option 2: manual startup

Open separate terminals and run:

```bash
yarpserver
```

```bash
cd "$VECTOR_VIEW"
gazebo worlds/robot.world
```

```bash
ISIRWholeBodyController --sequence StageTestTasks
```

Requires the full iCub stack from [Requirements](#run-the-full-icub-demo).

```bash
vectorGUI l_hand
```

### VectorGUI usage

```bash
# Short link name (builds the topic from default model context)
vectorGUI l_hand

# Full Gazebo transport path
vectorGUI /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact

# Optional robot name override (default: iCub)
vectorGUI l_hand iCub
```

Pass a link name (`l_hand`), not the sensor name (`l_hand_contact`). The program appends `_contact` automatically.

## Topic paths

Topic and collision names are built by `vectorview::TopicPath` in `src/common/TopicPath.cpp`.

**Plugin (from visual name):**

Given visual `iCub::l_hand::l_hand_visual`:

| Field | Value |
|-------|-------|
| Transport topic | `~/iCub/l_hand/l_hand_contact` |
| Collision scope | `iCub::l_hand::l_hand_collision` |

The `_visual` suffix is stripped to obtain the link name.

**GUI (from CLI argument):**

| Input | Result |
|-------|--------|
| `l_hand` | `/gazebo/default/iCub_fixed/iCub/l_hand/l_hand_contact` |
| Full path starting with `/` | Used as-is |

Defaults for short names live in `vectorview::ModelContext` (`include/vectorview/ModelContext.h`).

## Development

### Coding standards

First-party C++ follows the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines). Formatting is enforced with `.clang-format`:

```bash
./scripts/format.sh
```

Code under `third_party/` is excluded.

### CMake targets

| Target | Output |
|--------|--------|
| `vectorview` | `libvectorview.so` (Gazebo plugin) |
| `vectorGUI` | Qt executable |
| `vectorview_common` | Static library (shared utilities) |
| `vectorview_filters` | Shared library (force filter) |
| `dspfilters` | Vendored DSP library |
| `test_contact_utils`, `test_force_filter` | Unit test executables |

## License

Third-party components carry their own licenses (DSPFilters: MIT; QCustomPlot: GPL v3 or commercial). A project-level license file has not been added yet.
