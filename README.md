# Vector View

Visualize iCub contact forces in **Gazebo Sim (Harmonic)** on Ubuntu 24.04 Noble. This repository contains two subprojects that work together:

| Subproject | Binary | Role |
|------------|--------|------|
| **Vector View** | `libvector-view.so` | Gazebo Sim system plugin that draws a force arrow via marker messages |
| **Vector GUI** | `vector-gui` | Qt6 desktop app that displays contact data, plots force magnitude, and spawns models |

Both read from Gazebo transport contact topics. The plugin renders in the 3D view; the GUI provides a separate analysis window.

Bundled assets:

- `models/` contains the instrumented iCub model and simple spawn primitives
- `worlds/robot.world` is the demo Gazebo Sim world

## A little bit of history

Vector View began in the summer of **2015**, during a second-year engineering internship (*stage de fin de 2A*) at [ISIR](https://www.isir.upmc.fr/): the Institut des Systèmes Intelligents et de Robotique (Sorbonne Université / CNRS). The work took place in the **SYROCO** team (complex robotic systems) on the CentraleSupélec campus at **Gif-sur-Yvette**, within the broader [CoDyCo](https://www.codyco.eu/) research effort on whole-body contact control for the [iCub](https://icub.iit.it/) humanoid.

The goal was straightforward: make **contact forces visible in real time** in **Gazebo**, so researchers could see what the robot's hands and the control stack were doing during simulated interaction, instead of digging through log files. Two tools came out of that work:

- **Vector View**: a Gazebo plugin that draws a live force arrow on each instrumented link
- **Vector GUI**: a desktop app that plots force magnitude and spawns test objects into the running world

The work is described in a French internship report from October 2015. That document is not distributed with this repository.

The code from that summer shows it. Raw pointers, `using namespace` in headers, CMake 2.8, and architecture decisions best summarized as *"it compiled on my machine in 2015."* Eventually I could not open the tree without wincing; which is why [`docs/issues-report.md`](docs/issues-report.md) exists.

In **2026**, the project is being brought up to date for **Ubuntu 24.04 Noble**: **Gazebo Harmonic** instead of Classic, **Qt 6** instead of Qt4, and **C++17** throughout. Same research idea; a stack you can actually install today.

## Target stack (Ubuntu 24.04 Noble, 2026)

| Module | Target version |
|--------|----------------|
| OS | Ubuntu 24.04.3 LTS Noble |
| Gazebo | Harmonic (`gz-sim` 8.x) via `gz-harmonic` |
| Math / transport / msgs | `gz-math` 7.x, `gz-transport` 13.x, `gz-msgs` 10.x (Harmonic bundle) |
| Qt | Qt 6.4.2 |
| QCustomPlot | 2.1.1 |
| Boost | 1.83 |
| Protobuf | 3.21 |
| CMake | 3.16+ (3.28 on Noble) |
| C++ | C++17 |
| Catch2 | 2.13.10 (vendored; v3 optional) |
| DSPFilters | Vendored; no change required |

## Contact sensor naming

Each contact sensor publishes on an explicit topic (recommended) or on the Harmonic default path. This project uses short, stable topics:

```text
/vectorview/MODEL_INSTANCE/LINK_NAME
```

Example: link `l_hand` on model instance `iCub_fixed` uses `/vectorview/iCub_fixed/l_hand`.

The Vector View system plugin is attached at the **model** level (not on a visual). Topic names are resolved from the link name. See [Topic paths](#topic-paths) below.

## Repository layout

```text
include/vectorview/     Public headers
src/plugin/             Gazebo Sim system plugin
src/gui/                Qt6 application
src/filters/            Butterworth force filter wrapper
src/common/             Shared logic (ContactUtils, TopicPath, ContactMessage)
external/            Vendored DSPFilters, QCustomPlot 2.1.1, Catch2
models/                 Gazebo models
worlds/                 Gazebo Sim world files
scripts/                Demo launcher and format helper
tests/                  Unit tests (no Gazebo required)
.env.example            Environment variable template
docs/estimate.md        Modernization effort estimate vs. actual time
```

## Requirements

### Build the plugin and GUI

| Dependency | Notes |
|------------|-------|
| CMake 3.16+ | Build system |
| C++17 compiler | GCC 13+ or Clang |
| Gazebo Harmonic | `sudo apt install gz-harmonic` (OSRF repo on Noble) |
| Qt6 | `qt6-base-dev`, `libqt6opengl6-dev` |
| Protobuf, Boost | Pulled in by the Gazebo stack |

On **Ubuntu 24.04 Noble**, install OSRF Gazebo Harmonic and Qt6:

```bash
sudo apt update
sudo apt install curl lsb-release gnupg cmake g++ pkg-config \
  qt6-base-dev libqt6opengl6-dev

# Gazebo Harmonic (not in default Ubuntu repos)
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install gz-harmonic
```

### Run the full iCub demo

The original demo workflow also depends on external robotics software:

- [YARP](https://www.yarp.it/) and the iCub software stack
- [gz-sim-yarp-plugins](https://github.com/robotology/gz-sim-yarp-plugins) (replaces gazebo-yarp-plugins on Noble)
- [ocra-core](https://github.com/ocra-recipes/ocra-core)
- [codyco-superbuild](https://github.com/alexandrelheinen/codyco-superbuild) (provides `ISIRWholeBodyController`)

There is no single `apt install` for this layer. Follow the installation guides linked above.

### Bundled third-party code

Sources for [DSPFilters](https://github.com/vinniefalco/DSPFilters) and [QCustomPlot 2.1.1](http://www.qcustomplot.com/) are vendored under `external/`. You do not need to install them separately.

## Quick start

### Clone and build

```bash
git clone https://github.com/alexandrelheinen/vector-view.git
cd vector-view
mkdir build && cd build
cmake -DCMAKE_CXX_COMPILER=g++ ..
make
```

CMake options (all enabled by default):

| Option | Default | Purpose |
|--------|---------|---------|
| `BUILD_VECTORVIEW` | ON | Build the Vector View Gazebo Sim system plugin |
| `BUILD_VECTORGUI` | ON | Build the Vector GUI Qt6 application |
| `BUILD_TESTS` | ON | Build unit tests |

If Gazebo Harmonic or Qt6 is not found, CMake disables the plugin and GUI targets automatically and prints a warning. Tests still build.

Install to system paths (optional):

```bash
sudo make install
```

### Unit tests

Tests cover pure C++ logic and do not require Gazebo or Qt:

```bash
mkdir -p build && cd build
cmake -DCMAKE_CXX_COMPILER=g++ ..
make
ctest --output-on-failure
```

Force a tests-only configure:

```bash
cmake .. -DBUILD_VECTORVIEW=OFF -DBUILD_VECTORGUI=OFF -DBUILD_TESTS=ON
```

## Configuration

Gazebo Sim must locate the built plugin (`.so`) and bundled models:

```bash
cp .env.example .env
```

Edit `.env` if your paths differ from the defaults. The file is loaded by `scripts/run.sh`.

| Variable | Purpose |
|----------|---------|
| `VECTOR_VIEW` | Repository root (defaults to the directory above `scripts/`) |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | Directory containing `libvector-view.so` |
| `GZ_SIM_RESOURCE_PATH` | Directory containing bundled `models/` |
| `GZ_SIM_USER_PATH` | Directory containing bundled `worlds/` |
| `CODYCO_SUPERBUILD_ROOT` | Optional. Enables the ISIR controller step in `run.sh` |

Equivalent manual exports:

```bash
export VECTOR_VIEW=/path/to/vector-view
export PATH="$VECTOR_VIEW/build:$PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$VECTOR_VIEW/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$VECTOR_VIEW/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$VECTOR_VIEW/worlds:${GZ_SIM_USER_PATH:-}"
```

## Running the demo

### Option 1: run script

```bash
chmod +x scripts/run.sh
./scripts/run.sh
```

The script:

1. Sources `.env` when present
2. Exports Gazebo Sim paths
3. Opens YARP, `gz sim`, two Vector GUI instances, and (if configured) the ISIR controller in separate terminal tabs

If `CODYCO_SUPERBUILD_ROOT` is unset, the controller step is skipped.

### Option 2: manual startup

```bash
yarpserver
```

```bash
cd "$VECTOR_VIEW"
gz sim -r worlds/robot.world
```

```bash
ISIRWholeBodyController --sequence StageTestTasks
```

Requires the full iCub stack from [Run the full iCub demo](#run-the-full-icub-demo).

```bash
vector-gui l_hand
```

### Vector GUI usage

```bash
# Short link name (builds the topic from default model context)
vector-gui l_hand

# Full transport path
vector-gui /vectorview/iCub_fixed/r_hand

# Optional robot name override (default: iCub)
vector-gui l_hand iCub

# Optional world name for spawn service (default: default)
vector-gui l_hand iCub default
```

Pass a link name (`l_hand`), not the sensor name (`l_hand_contact`). Short names map to `/vectorview/iCub_fixed/l_hand`.

## Topic paths

Topic and collision names are built by `vectorview::TopicPath` in `src/common/TopicPath.cpp`.

**Plugin (from link name in SDF):**

| Field | Value |
|-------|-------|
| Transport topic | `/vectorview/iCub_fixed/l_hand` |
| Collision scope | `iCub::l_hand::l_hand_collision` |

**GUI (from CLI argument):**

| Input | Result |
|-------|--------|
| `l_hand` | `/vectorview/iCub_fixed/l_hand` |
| Full path starting with `/` | Used as-is |

Defaults for short names live in `vectorview::ModelContext` (`include/vectorview/ModelContext.h`).

## Development

### Coding standards

First-party C++ follows the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines). Formatting is enforced with `.clang-format`:

```bash
./scripts/format.sh
```

Code under `external/` is excluded.

### CMake targets

| Target | Output | Subproject |
|--------|--------|------------|
| `vectorview` | `libvector-view.so` | Vector View (Gazebo Sim system plugin) |
| `vector-gui` | Qt6 executable | Vector GUI |
| `vectorview_common` | Static library | Shared utilities |
| `vectorview_filters` | Shared library | Force filter |
| `dspfilters` | Vendored DSP library | External dependency |
| `test_contact_utils`, `test_force_filter` | Unit test executables | Tests |

## Migration from Gazebo Classic

v2.0 replaces the Gazebo Classic visual plugin and Qt4 GUI with a Harmonic system plugin and Qt6 GUI. See `docs/estimate.md` for the modernization scope and effort notes.

## License

Third-party components carry their own licenses (DSPFilters: MIT; QCustomPlot: GPL v3 or commercial). A project-level license file has not been added yet.
