# Example Source Layout

The source tree is grouped by the subsystem or workflow each example demonstrates.
Executable target names are intentionally stable even when source files move.

## Running examples

Build from the `raisim2Lib` root:

```bash
cmake -S . -B build-examples \
  -DRAISIM_EXAMPLE=ON \
  -DRAISIM_CHECK_FOR_UPDATES=OFF
cmake --build build-examples -j
cd build-examples/examples
```

Most `server/*` examples publish to `raisim::RaisimServer`. Start
`<raisim-install>/bin/rayrai_raisim_tcp_viewer` in another terminal to visualize them.
Most `rayrai/*` examples create an in-process rayrai window and do not need the
TCP viewer.

## Directory groups

- `server/basics`: primitive objects, dense sphere-drop scenes, mesh objects,
  compound objects, dynamic object addition, and YCB object loading.
- `server/assets`: model import, mesh preprocessing, cache reuse, `addMesh`
  workflows, and mesh asset export.
- `server/materials`: contact material effects such as restitution and static
  friction.
- `server/terrain`: procedural, image, and dynamic height map examples.
- `server/sensors`: CPU ray casting and ray-scan lidar examples.
- `server/robots`: articulated robot examples and robot control workflows.
- `server/dynamics`: constraints, moving platforms, springs, and rigid-body
  dynamics demos.
- `server/visualization`: server-driven visual output and synchronous update
  flow.
- `server/performance`: object lifecycle stress and island sleeping examples.
- `server/deformable`: cloth, surface-mesh deformables, filled deformables,
  internal struts, compliance, and Young's modulus examples.
- `server/mjcf`: MJCF world loading and articulated Gymnasium models.
- `benchmark`: timing-oriented examples for common RaiSim workloads.
- `rayrai/getting_started`: minimal and complete rayrai entry points.
- `rayrai/sensors`: rayrai RGB camera, depth camera, lidar point cloud, and
  marker examples.
- `rayrai/visuals`: custom visuals, instancing, and point cloud examples.
- `rayrai/dynamics`: in-process rigid-body dynamics visualizations.
- `rayrai/materials`: PBR and texture material examples.
- `rayrai/assets`: mesh and asset-processing visualization examples, including
  visual-only glTF assets kept separate from collision meshes.
- `rayrai/runtime`: runtime scene editing using stable object ids, snapshots,
  collision filters, cloning, and removal.
- `rayrai/collision`: collision detection examples such as swept continuous
  collision detection.
- `rayrai/tools`: standalone rayrai tools such as the TCP viewer.
- `worlds`: larger packaged scene examples.
- `xml`: XML world loading and templated XML world examples.

## Useful starting targets

- `primitive_grid`: basic server-side simulation and visualization.
- `sphere_drop`: drop 1000 dynamic spheres onto a ground plane.
- `rayrai_basic_scene`: minimal in-process rayrai rendering.
- `rayrai_complete_showcase`: broad rayrai feature overview.
- `rayrai_depth_camera`: rayrai depth capture plus CPU depth-camera comparison.
- `rayrai_rgb_camera`: in-process rayrai RGB capture.
- `rayrai_rolling_spinning_friction`: rolling and spinning friction on a grid
  of spheres and cylinders.
- `deformable_objects`: cloth, mesh deformables, filled particles, struts,
  compliance, and elastic modulus.
- `model_asset_pipeline`: mesh preprocessing and OBJ asset export.
- `mjcf_gymnasium_hopper`: load and actuate Gymnasium's Hopper MJCF model.
- `mjcf_gymnasium_walker2d`: load and actuate Gymnasium's Walker2d MJCF model.
- `mjcf_gymnasium_humanoid`: load Gymnasium's Humanoid MJCF model and drop it
  from a raised arbitrary configuration.
- `anymal_standing_benchmark`: run the native ANYmal PD-standing benchmark and
  report simulation throughput plus average contacts.
- `articulated_system_benchmark`: run standalone timing scenes for ANYmal,
  Atlas, and chain articulated systems.
- `dynamic_heightmap`: animate a heightmap and color map through RaisimServer.
- `rayrai_coacd_mesh_approximation`: original mesh versus CoACD convex approximation mesh
  collision parts through `World::addMesh`.
- `rayrai_visual_asset_support`: inspect realistic textured URDF assets while
  keeping visual and collision geometry separate.
- `rayrai_runtime_scene_editing`: stable ids, snapshots, collision filters,
  cloning, and removal.
- `rayrai_swept_ccd`: swept CCD settings for a fast falling sphere.

Some targets are guarded by installed RaiSim API availability. If CMake prints a
`Skipping ...` message, install a newer RaiSim/rayrai package and reconfigure.
