####################################
Server Example: Shadow Hand USD Cube
####################################

Overview
========
Loads NVIDIA Isaac Sim's Shadow Robot ShadowHand USD scene directly with
``World(shadow_hand.usd)`` and places a RaiSim rigid-body cube at the Isaac Lab
in-hand cube start pose. The linked RaiSim package is expected to expose USD
scene loading; the constructor imports the USD Physics rigid bodies and joints
as a RaiSim articulated system.

The cube is a native RaiSim box for physics and a textured OBJ visual proxy for
rendering, so the example does not redistribute the separate NVIDIA DexCube prop
asset.

Build Availability
==================
The executable is generated only when CMake finds a RaiSim package with USD
scene loading. RaiSim is expected to include OpenUSD on every supported
architecture.

Binary
======
Installed executable when available: ``shadow_hand_usd_cube``.

Run
===
Run the installed executable:

.. code-block:: bash

   <raisim-install>/bin/shadow_hand_usd_cube

On Windows, run ``shadow_hand_usd_cube.exe`` instead.
This example uses RaisimServer. Start ``rayrai_raisim_tcp_viewer`` and connect
to port 8080.

For a non-visual load test, run:

.. code-block:: bash

   <raisim-install>/bin/shadow_hand_usd_cube --headless-test

Details
=======
- Loads ``rsc/isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd`` with
  the ``World`` constructor.
- Uses RaiSim's bundled OpenUSD runtime directly; no Assimp USD importer or
  USD-specific build switch is required.
- Places the ShadowHand base at ``(0, 0, 0.5)``.
- Uses a primitive static proxy hand only if the first-pass importer finds no
  supported primitive collision shapes.
- Creates a dynamic cube at ``(0, -0.39, 0.6)``, matching the Isaac Lab Shadow
  Hand cube task's initial object pose.
- Adds ``rsc/isaac/Props/Blocks/DexCube/dex_cube_textured.obj`` as a textured
  visual mesh and synchronizes it to the dynamic cube each step.
- Adds a translucent visual goal cube above the hand.
