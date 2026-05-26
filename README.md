# RaiSim2

<img src="docs/image/rayrai_complete_showcase.png" alt="rayrai complete showcase" width="100%">

<img src="docs/image/granular_media.png" alt="granular media example" width="100%">

<img src="docs/image/deformable_objects.png" alt="deformable objects example" width="100%">

<img src="docs/image/procedural_heightmap.png" alt="procedural heightmap example" width="100%">

RaiSim is a physics engine for robotics and artificial intelligence research. The public distribution is provided as binary packages with headers, libraries, examples, rayrai tools, and documentation.

[![RaiSim video](https://img.youtube.com/vi/CN0ah5-OWik/0.jpg)](https://www.youtube.com/watch?v=CN0ah5-OWik)

## Install

Download the binary package for your platform and unpack it to an install location such as `$HOME/raisim2Lib` on Linux/macOS or `C:\raisim` on Windows. Keep the package directories together; examples and rayrai tools expect the bundled assets to remain next to the installed binaries.

Set up the runtime environment before running examples or applications. From the unpacked package root:

```bash
cd $HOME/raisim2Lib
source ./raisim_env.sh
export RAISIM_LOCAL_INSTALL_ROOT=$PWD
```

`raisim_env.sh` must be sourced, not executed. It sets `RAISIM_DIR` and `RAISIM_OS`, then prepends `raisim/lib` and `rayrai/lib` to `LD_LIBRARY_PATH` on Linux or `DYLD_LIBRARY_PATH` on macOS.

On Windows, run one of the provided environment scripts from the package root:

```powershell
.\raisim_env.ps1
```

or:

```bat
raisim_env.bat
```

The Windows scripts prepend the installed RaiSim and rayrai `bin` and `lib` directories to `Path`.
## Build Examples With CMake

From the repository root, configure and build the C++ examples:

```bash
cmake -S . -B build-examples \
  -DCMAKE_BUILD_TYPE=Release \
  -DRAISIM_EXAMPLE=ON \
  -DRAISIM_CHECK_FOR_UPDATES=OFF
cmake --build build-examples --parallel
```

`RAISIM_EXAMPLE` is enabled by default, but the flag is shown explicitly. Keep `RAISIM_CHECK_FOR_UPDATES=OFF` when you want to build against the package already present in this checkout; omit it to let CMake check for newer releases.

Before running source-built examples from the same checkout, source the environment script from the repository root:

```bash
source ./raisim_env.sh
```

On Windows, run `.\raisim_env.ps1` or `raisim_env.bat` before launching examples, and add `--config Release` to the build and install commands.

The example executables are written to `build-examples/examples` on Linux/macOS and `build-examples/bin` on Windows:

```bash
./build-examples/examples/rayrai_tcp_viewer
./build-examples/examples/primitive_grid
```

Install headers, libraries, CMake package files, and bundled rayrai tools to a local prefix with:

```bash
cmake --install build-examples --prefix "$HOME/.local"
```

The install target does not install source-built example executables; run those from the build tree. Downstream projects using this CMake-installed layout should pass `-DCMAKE_PREFIX_PATH=$HOME/.local`.

## Activation

Please go to [raisim.com](https://raisim.com) to get your raisim license.
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

### Rayrai TCP viewer

<img src="docs/image/rayrai/tcp_viewer/tcp_viewer_overview.png" alt="rayrai TCP viewer" width="100%">

`rayrai_raisim_tcp_viewer` is the recommended visualizer for `RaisimServer` simulations. It connects to a running server over TCP and renders the world with the full rayrai PBR pipeline (procedural sky, IBL, directional shadows, reflective ground, weather, tone mapping). The overlay surfaces:

* **Pause / Step / Step 10** — interactive simulation control over the wire (`PROTOCOL_FEATURE_SIM_CONTROL`); pause and step a remote server from the viewer without changing simulation code.
* **Force / pose / generalized-coordinate authoring** — drag-poke objects, retarget single-body poses, or override an articulated system's joint angles directly from the inspector.
* **Render tab** — switch render-quality presets, tone mapping, weather, bloom, SSAO, and DoF at runtime.
* **Drag-drop joint inspector** — drop a URDF or MJCF file on the disconnected viewer to inspect joints, limits, and mesh resolution kinematically.
* **Headless screenshots and session record/replay** — `--screenshot`, `--record-session`, and `--replay-session` for CI, doc image generation, and offline replays.

See the documentation page on the rayrai TCP viewer for the full panel walkthrough, the wire format, and a minimal custom-client example.

## Documentation

Open the installed documentation from the package, or visit the hosted RaiSim documentation for installation, activation, API, examples, and visualization workflows.
