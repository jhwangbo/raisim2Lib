# RaiSim2

<img src="docs/image/rayrai_complete_showcase.gif" alt="rayrai_complete_showcase" width="100%">

<img src="docs/image/granular_media.gif" alt="granular_media" width="100%">

<img src="docs/image/deformable_objects.gif" alt="deformable_objects" width="100%">

<img src="docs/image/procedural_heightmap.gif" alt="procedural_heightmap" width="100%">

RaiSim is a physics engine for robotics and artificial intelligence research. The public distribution is provided as binary packages with headers, libraries, examples, rayrai tools, and documentation.

[![RaiSim video](https://img.youtube.com/vi/CN0ah5-OWik/0.jpg)](https://www.youtube.com/watch?v=CN0ah5-OWik)

## Install

Download the binary package for your platform and unpack it to an install location such as `$HOME/raisim2Lib` on Linux/macOS or `C:\raisim` on Windows. Keep the package directories together; examples and rayrai tools expect the bundled assets to remain next to the installed binaries.

Set the runtime library path before running examples or applications:

```bash
export RAISIM_LOCAL_INSTALL_ROOT=$HOME/raisim2Lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RAISIM_LOCAL_INSTALL_ROOT/raisim/lib
```

On macOS, use `DYLD_LIBRARY_PATH`. On Windows, add the installed RaiSim `bin` directory to `Path`.

## Activation

Rename the activation key received by email to `activation.raisim` and place it in the default location:

```text
Linux/macOS: $HOME/.raisim/activation.raisim
Windows:     C:\Users\<YOUR-USERNAME>\.raisim\activation.raisim
```

You can also set the activation key explicitly in your application with `raisim::World::setActivationKey()`.

## Run Examples

Server-based examples publish a `raisim::World` through `RaisimServer`. Start the rayrai TCP viewer first, then run an example:

```bash
$RAISIM_LOCAL_INSTALL_ROOT/bin/rayrai_raisim_tcp_viewer
$RAISIM_LOCAL_INSTALL_ROOT/bin/example_anymal_contacts
```

In-process rayrai examples open their own renderer window and do not need the TCP viewer:

```bash
$RAISIM_LOCAL_INSTALL_ROOT/bin/example_rayrai_pbr_asset_inspector
$RAISIM_LOCAL_INSTALL_ROOT/bin/example_polyhaven_blue_wall --fast-load
```

## Use RaiSim From C++

RaiSim is installed as a CMake package. Point `CMAKE_PREFIX_PATH` at the installed package prefix from your downstream project:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=$RAISIM_LOCAL_INSTALL_ROOT/raisim
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

Use `rayrai_raisim_tcp_viewer` for applications that publish through `raisim::RaisimServer`. Use in-process rayrai APIs when your application needs its own renderer window, offscreen rendering, RGB/depth sensors, or screenshots. RaisimUnity and RaisimUnreal are legacy integrations and are no longer the supported visualization path.

## Documentation

Available at raisim.com
