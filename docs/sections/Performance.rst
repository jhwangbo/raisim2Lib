#############################
Performance
#############################

This page is a practical tuning guide for the current source tree. For commands
that build and run benchmarks, see :doc:`BuildAndTest`.

Fast scenes usually come from three choices:

* keep collision geometry simple and intentional;
* avoid creating more contacts than the task needs;
* measure with the benchmark runner on one thread before changing solver or
  modeling choices.

What Usually Costs Time
=======================

.. list-table::
   :header-rows: 1
   :widths: 24 38 38

   * - Cost center
     - What increases it
     - What to do first
   * - Broadphase collision detection
     - Many active collision bodies, large worlds, or dense moving object sets.
     - Use collision groups/masks and choose a broadphase that matches the scene.
       See :doc:`WorldSystem` and :doc:`CollisionDetection`.
   * - Narrowphase collision detection
     - Expensive shape pairs, dense mesh contact, or geometry that creates many
       candidate contacts.
     - Use primitives, height maps, simplified convex collision assets, or cached
       mesh preprocessing where possible. See :doc:`SingleBodyObjects`.
   * - Contact properties
     - Many articulated bodies touching many other bodies.
     - Remove unnecessary collision shapes and split non-interacting objects with
       collision masks.
   * - Contact solver
     - Stacks, pinched contacts, highly coupled contacts, or excessive contact
       manifolds.
     - Reduce redundant contacts before tuning solver iterations.
   * - Sensors
     - Large CPU depth images, many lidar rays, or high sensor update rates.
     - Use in-process rayrai for RGB/depth sensor images when rayrai is
       available. Lower CPU ray-query resolution/update rate only for headless
       fallback or deterministic physics-ray workloads. See :doc:`Sensors`.
   * - Visualization
     - Rendering in the simulation process, high-quality PBR settings, or
       synchronous screen capture.
     - Use ``RaisimServer`` + ``rayrai_tcp_viewer`` for debugging, and
       in-process rayrai only when the application needs rendered images. See
       :doc:`Visualization`.

Collision Modeling
==================

Collision geometry is often the largest performance lever. Visual meshes should
not automatically become collision meshes. Prefer this order:

1. primitive shapes for robot links and simple objects;
2. height maps for terrain;
3. preprocessed convex collision assets for irregular objects;
4. mesh collision only when the task really needs mesh-level contact.

Use ``World::addMesh`` preprocessing and cache reuse when a mesh collision asset
is needed repeatedly. For broadphase-heavy scenes, configure
``contact::BroadphaseType::MultiBoxPrune`` with bounds and cell sizes that match
the active world volume. For very small scenes or debugging, brute-force
broadphase can be useful, but it should not be the default assumption for large
worlds.

Contacts and Solver Work
========================

The contact solver cost depends on how coupled the contacts are. Ten isolated
contacts are very different from ten contacts in a compressed stack. Before
tuning solver parameters, check whether the model is generating contacts that do
not matter for the task:

* remove decorative collision bodies;
* simplify foot, gripper, and terrain collision geometry;
* avoid overlapping collision shapes in one body unless they are intentional;
* use collision groups and masks to skip pairs that should never interact;
* keep timesteps and material parameters within the stability range required by
  the task.

``World::setContactSolverParam`` exposes solver parameters for advanced users,
but it is usually a later step. Reducing redundant contacts tends to be more
robust than asking the solver to process a harder problem faster.

Sleeping Islands
================

Sleeping is enabled by default. RaiSim can skip simulation work for dynamic
islands whose velocities remain below the configured thresholds for a few steps.
This helps scenes with piles, props, or objects that settle and then stay quiet.

.. code-block:: cpp

  world.setSleepingEnabled(true);
  world.setSleepingParameters(/*linear*/ 0.002, /*angular*/ 0.01, /*quietSteps*/ 2);
  world.wakeAll();

Disable sleeping when every object must remain numerically active every step, or
when a benchmark is intended to measure the awake dynamics path. Use the
``island_sleep`` benchmark to quantify the effect for stack-like scenes.

Benchmark Workflow
==================

Build benchmarks with ``RAISIM_BENCHMARK=ON`` and run timing on one thread:

.. code-block:: bash

  cmake -S . -B build-benchmark \
    -DCMAKE_BUILD_TYPE=Release \
    -DRAISIM_BENCHMARK=ON \
    -DRAISIM_TEST=OFF
  cmake --build build-benchmark --target benchmarks -j12
  ./build-benchmark/bin/benchmarks --repeat 3

Representative benchmark IDs include:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Benchmark
     - Use it for
   * - ``world_integration``
     - End-to-end stepping cost for a mixed world.
   * - ``anymal_standing`` and ``anymal_falling``
     - Articulated robot contact scenarios.
   * - ``chain10_speed`` and ``chain20_speed``
     - Articulated dynamics scaling with little collision work.
   * - ``primitives`` and ``narrowphase``
     - Primitive collision and narrowphase contact generation.
   * - ``mesh_collider_speed`` and ``mesh_stack_plane``
     - Mesh collision and mesh stack behavior.
   * - ``heightmap_lidar`` and ``depth_camera``
     - CPU fallback sensor workloads; prefer rayrai sensor rendering for
       RGB/depth observations when renderer output is available.
   * - ``granular_dense_contact`` and ``granular_heightmap``
     - Granular contact workloads.
   * - ``deformable_cloth`` and ``deformable_cube_stack``
     - Deformable solver and contact workloads.
   * - ``island_sleep``
     - Sleeping-island speedup and wake behavior.

Use benchmark-specific help to inspect options:

.. code-block:: bash

  ./build-benchmark/bin/benchmarks --bench world_integration -- --help

For stable comparisons, run the same executable, compiler, build type, CPU
governor, and benchmark arguments. Avoid comparing a visualized run against a
headless run unless visualization is the measured workload.

Threading and Determinism
=========================

RaiSim simulation performance should be measured on one simulation thread unless
the experiment explicitly studies external parallelism. Running many worlds in
parallel is an application-level design choice; it is different from changing the
deterministic stepping behavior of one ``raisim::World``.

For deterministic contact-solver iteration order, use
``World::setContactSolverIterationOrder``. For reproducible benchmark numbers,
also control random seeds in the benchmark or application, keep visualization
disabled unless measured, and pin the same benchmark arguments.

Algorithm Background
====================

RaiSim uses an articulated-system dynamics path designed to avoid forming and
inverting a full mass matrix for ordinary forward dynamics. The practical effect
is that articulated dynamics without heavy contact work scales much better with
degrees of freedom than a pipeline that explicitly forms and factorizes the mass
matrix every step.

That does not make every simulation linear in model size. Collision detection,
contact generation, solver coupling, deformable bodies, granular media, sensors,
and rendering can dominate the wall time. Treat old headline kHz numbers as
historical context, not as a guarantee for a current scene. The benchmark runner
is the source of truth for the model, compiler, hardware, and build options in
front of you.
