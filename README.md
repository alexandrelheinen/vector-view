# VectorView and VectorGUI

**VectorView** (`libvectorview.so`) is a Gazebo visual plugin that draws a force arrow on an iCub link visual, using data from the link's contact sensor.

**VectorGUI** (`vectorGUI`) is a Qt desktop application that subscribes to the same contact topics, shows contact information, plots force magnitude, and can spawn simple models in simulation.

Bundled Gazebo models live under `models/`. The demo world is `worlds/robot.world`.

#### Contact sensor naming

Contact sensors must follow `LINK_NAME_contact` (for example `l_hand_contact` on link `l_hand`).

## Project layout

```
include/vectorview/     First-party public headers
src/plugin/             Gazebo visual plugin
src/gui/                Qt desktop application
src/filters/            ForceFilter wrapper
src/common/             Shared pure C++ utilities (ContactUtils, TopicPath)
third_party/            Vendored DSPFilters, QCustomPlot, and Catch2
models/                 Gazebo models (iCub, spawn primitives)
worlds/                 Gazebo world files
scripts/                Demo launcher, formatting helper, env bootstrap
tests/                  Unit tests (no Gazebo required)
.env.example            Template for local environment variables
```

## Dependencies

This project has two dependency tiers.

### Tier A — build VectorView / VectorGUI

Required to compile the plugin and GUI on a **legacy Gazebo Classic + Qt4** stack:

- CMake >= 3.10
- C++ compiler with C++11 support
- pkg-config, Protobuf, Boost (system, filesystem)
- Gazebo Classic (pkg-config module `gazebo`)
- OGRE 1.9
- Qt4 (QtGui, QtXml, QtCore)

Example on **Ubuntu 20.04** (adjust names for your distro/Gazebo version):

```bash
sudo apt update
sudo apt install cmake g++ pkg-config \
  libgazebo-dev libogre-1.9-dev libprotobuf-dev \
  libboost-system-dev libboost-filesystem-dev \
  libqt4-dev libqt4-opengl-dev
```

On modern distros such as Ubuntu 22.04/24.04, **Qt4 and Gazebo Classic are usually unavailable from apt**. In that case you can still build and run the unit tests (see below).

Vendored libraries:

- [DSPFilters](https://github.com/vinniefalco/DSPFilters)
- [QCustomPlot](http://www.qcustomplot.com/)

Their sources are included under `third_party/`.

### Tier B — run the full iCub demo

The original internship demo additionally requires:

- [YARP](https://www.yarp.it/) and iCub tooling
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [ocra-core](https://github.com/ocra-recipes/ocra-core) and [codyco-superbuild](https://github.com/alexandrelheinen/codyco-superbuild) for `ISIRWholeBodyController`

These cannot be installed with a single `apt` line. Follow the upstream installation guides for those projects.

## Coding standards

First-party C++ code follows the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines).

Formatting is defined by `.clang-format`. Run:

```bash
./scripts/format.sh
```

Vendored code under `third_party/` is excluded from formatting.

## Build

```bash
git clone https://github.com/alexandrelheinen/vector-view.git
cd vector-view
mkdir build && cd build
cmake ..
make
```

If Gazebo or Qt4 are missing, CMake disables `BUILD_VECTORVIEW` and `BUILD_VECTORGUI` and keeps `BUILD_TESTS` enabled.

Optional install:

```bash
sudo make install
```

## Unit tests

Pure logic tests run without Gazebo or Qt:

```bash
mkdir build && cd build
cmake ..
make
ctest --output-on-failure
```

Explicit tests-only configure:

```bash
cmake .. -DBUILD_VECTORVIEW=OFF -DBUILD_VECTORGUI=OFF -DBUILD_TESTS=ON
```

## Environment setup

Gazebo needs to find the built plugin and bundled models. Instead of exporting variables manually in every shell, copy the template and let the run script load it:

```bash
cp .env.example .env
# edit .env if your paths differ
```

`.env` is **not automatic by itself** — shells do not load it unless you `source` it or use `scripts/run.sh`, which sources `.env` when present.

Variables used by the project:

```bash
export VECTOR_VIEW=/path/to/vector-view
export PATH=$VECTOR_VIEW/build:$PATH
export GAZEBO_PLUGIN_PATH=$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH:-}
export GAZEBO_MODEL_PATH=$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH:-}
```

Optional for the full demo:

```bash
export CODYCO_SUPERBUILD_ROOT=/path/to/codyco-superbuild
```

## Run the demo

### Manual startup

In separate terminals:

```bash
yarpserver
cd "$VECTOR_VIEW" && gazebo worlds/robot.world
ISIRWholeBodyController --sequence StageTestTasks   # requires Tier B setup
vectorGUI l_hand                                    # or pass the full topic path
```

Short-name CLI examples:

```bash
vectorGUI l_hand
vectorGUI /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact
vectorGUI l_hand iCub                               # optional robot-name override
```

### Shell script

```bash
chmod +x scripts/run.sh
./scripts/run.sh
```

The script sources `.env`, exports Gazebo paths, and launches the demo processes in `gnome-terminal` tabs. It skips the ISIR controller step if `CODYCO_SUPERBUILD_ROOT` is unset.

## Topic path logic

Topic names are built by `vectorview::TopicPath`:

- the plugin derives link-based topics from a visual name such as `iCub::l_hand::l_hand_visual`
- the GUI accepts either a full transport path or a short link name such as `l_hand`

This avoids ad hoc string concatenation spread across the plugin and GUI entry point.
