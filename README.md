# RaiSim2

<img src="docs/image/rayrai_complete_showcase.gif" alt="rayrai_complete_showcase" width="100%">

<img src="docs/image/granular_media.gif" alt="granular_media" width="100%">

<img src="docs/image/deformable_objects.gif" alt="deformable_objects" width="100%">

<img src="docs/image/procedural_heightmap.gif" alt="procedural_heightmap" width="100%">

RaiSim is a physics engine for robotics and artificial intelligence research.
RaiSim2 is distributed as binary libraries with headers, examples, rayrai tools,
and documentation. Get the public package from
https://github.com/raisimTech/raisim2Lib.

[![RaiSim video](https://img.youtube.com/vi/CN0ah5-OWik/0.jpg)](https://www.youtube.com/watch?v=CN0ah5-OWik)

## Install

Download the RaiSim2 binary package for your platform from
https://github.com/raisimTech/raisim2Lib and unpack it to an install location
such as `$HOME/raisim2Lib` on Linux/macOS or `C:\raisim` on Windows. Keep the
package directories together; examples and rayrai tools expect the bundled
assets to remain next to the installed binaries.

Set the runtime library path before running examples, rayrai tools, RaiSim
applications, or `raisimPy`:

```bash
cd $HOME/raisim2Lib
source ./raisim_env.sh
```

The environment script adds both RaiSim and rayrai libraries to the platform
loader path. On Windows, use `raisim_env.bat` or add the installed RaiSim and
rayrai `bin` directories to `Path`.

## Build Examples And raisimPy

From the `raisim2Lib` root, configure and build the examples and Python wrapper:

```bash
cmake -S . -B build \
  -DRAISIM_EXAMPLE=ON \
  -DRAISIM_PY=ON
cmake --build build -j
```

`RAISIM_EXAMPLE` is enabled by default, but it is shown here because most users
build examples first. `RAISIM_PY` is disabled by default and must be enabled to
build `raisimPy`.

Installation is optional for running examples from the build tree. If you do
install, choose a prefix you can write to instead of relying on CMake's default
`/usr/local`:

```bash
cmake --install build --prefix $HOME/raisim2Lib/install
```

## Activation

Rename the activation key received by email to `activation.raisim` and place it in the default location:

```text
Linux/macOS: $HOME/.raisim/activation.raisim
Windows:     C:\Users\<YOUR-USERNAME>\.raisim\activation.raisim
```

You can also set the activation key explicitly in your application with `raisim::World::setActivationKey()`.

## Run Examples

Server-based examples publish a `raisim::World` through `RaisimServer`. Start
the rayrai TCP viewer in one sourced terminal:

```bash
source ./raisim_env.sh
./build/examples/rayrai_tcp_viewer
```

Run the example in another sourced terminal:

```bash
source ./raisim_env.sh
./build/examples/primitive_grid
```

In-process rayrai examples open their own renderer window and do not need the TCP viewer:

```bash
source ./raisim_env.sh
./build/examples/rayrai_basic_scene
./build/examples/rayrai_complete_showcase
```

## Use RaiSim From C++

RaiSim and rayrai are CMake packages in the unpacked `raisim2Lib` tree. Point
`CMAKE_PREFIX_PATH` at those package prefixes from your downstream project:

```bash
export RAISIM_ROOT=$HOME/raisim2Lib
cmake -S . -B build -DCMAKE_PREFIX_PATH="$RAISIM_ROOT/raisim;$RAISIM_ROOT/rayrai"
cmake --build build -j
```

Minimal downstream `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_raisim_app LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(app main.cpp)
target_link_libraries(app PRIVATE raisim::raisim)
if (UNIX)
  target_link_libraries(app PRIVATE pthread)
endif()
```

## Visualization

Use `rayrai_tcp_viewer` for applications that publish through `raisim::RaisimServer`. Use in-process rayrai APIs when your application needs its own renderer window, offscreen rendering, RGB/depth sensors, or screenshots. RaisimUnity and RaisimUnreal are legacy integrations and are no longer the supported visualization path.

## Documentation

Available at raisim.com
