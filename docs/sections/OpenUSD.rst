#############################
OpenUSD Loading
#############################

RaiSim loads OpenUSD files directly through the bundled OpenUSD runtime.
OpenUSD support is part of supported RaiSim packages and source builds; it is
not an optional Assimp importer path and there is no CMake switch to build
RaiSim without it.

Supported files
===============
The mesh loader accepts ``.usd``, ``.usda``, ``.usdc``, and ``.usdz`` files
wherever a RaiSim mesh path is accepted:

.. code-block:: cpp

    raisim::World world;
    auto* mesh = world.addMesh("asset.usd",
                               1.0,
                               1.0,
                               "default",
                               raisim::MeshCollisionMode::ORIGINAL_MESH);

The same OpenUSD-backed path is used by ``raisim::Mesh::loadMesh`` and
``raisim::Mesh::preprocessMesh``. Use ``preprocessMesh`` when you want a
deterministic triangulated OBJ cache that can be reused by later runs.

Source builds with USD scene loading can also read USD Physics rigid bodies and
fixed/revolute/prismatic joints directly through the world constructor:

.. code-block:: cpp

    raisim::World world("robot.usd");

Runtime requirements
====================
Installed packages include the OpenUSD runtime next to RaiSim:

* On Linux, the runtime is installed under ``raisim/lib/openusd`` and the
  package environment script adds the required library path.
* On Windows, the USD DLLs are installed next to the RaiSim binaries and the
  plugin resources are under ``raisim/bin/openusd``.

Keep the ``bin``, ``lib``, ``openusd``, and ``rsc`` directories together when
copying an installed package. If you build from source, the bundled prebuilt
OpenUSD tree must exist under ``prebuilt/openusd/<platform>``; CMake fails at
configure time if the required OpenUSD headers or libraries are missing.

Import semantics
================
The mesh APIs import USD as triangle mesh geometry:

* The loader opens the file as a ``UsdStage`` and traverses ``UsdGeomMesh``
  prims.
* Parent transforms are applied to each mesh before vertices are added.
* Polygon faces are triangulated, and left-handed USD mesh winding is reversed.
* Multiple mesh prims in one USD file are merged into one RaiSim mesh object.

The articulated-system API reads ``UsdPhysicsRigidBodyAPI`` bodies and
``PhysicsFixedJoint``, ``PhysicsRevoluteJoint``, and ``PhysicsPrismaticJoint``
relationships. It maps primitive cube/sphere/capsule/cylinder collision shapes
and ignores PhysX tendons, drives, variants, skeletons, lights, and full
material graphs. For high-detail render assets, keep a separate simplified
collision mesh or use ``MeshCollisionMode::CONVEX_HULL`` /
``MeshCollisionMode::CONVEXIFY`` when appropriate.

Examples
========
``shadow_hand_usd_cube`` loads
``rsc/isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd`` through
``World(shadow_hand.usd)`` and publishes the scene through ``RaisimServer``.
The example target is generated only when CMake finds a RaiSim package with USD
scene loading. RaiSim is expected to include OpenUSD on every supported
architecture. Start the TCP viewer, then run:

.. code-block:: bash

    <raisim-install>/bin/rayrai_raisim_tcp_viewer
    <raisim-install>/bin/shadow_hand_usd_cube

``nvidia_usd_robots`` provides additional vetted Isaac Sim robot scenes:
``create3``, ``jetbot``, and ``ant``.

.. code-block:: bash

    <raisim-install>/bin/nvidia_usd_robots --asset create3

On Windows, use the corresponding ``.exe`` binaries.

rayrai can also load USD files as visual-only meshes through
``RayraiWindow::addVisualMesh``. This is useful for inspection, but the same
scope applies: geometry, transforms, basic display colour/opacity, not full USD
scene semantics.

Troubleshooting
===============
If a USD asset fails to load:

* Verify that the asset path exists and that the package ``rsc`` directory was
  copied with the binaries.
* Run the package environment script before launching examples from outside the
  installed ``bin`` directory.
* On Windows, make sure the USD DLLs are beside the executable and
  ``bin/openusd`` is still present.
* If source configuration fails, regenerate or restore the matching bundled
  OpenUSD prebuilt runtime for your platform.
