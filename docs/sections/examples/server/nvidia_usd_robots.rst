################################
Server Example: NVIDIA USD Robots
################################

Overview
========
Loads a selected NVIDIA Isaac Sim robot USD scene directly with the
``World(<asset>.usd)`` constructor and publishes it through ``RaisimServer``.
The example is intentionally limited to assets that were smoke-tested as RaiSim
articulated systems with supported collision bodies.

Build Availability
==================
The executable is generated only when CMake finds a RaiSim package with USD
scene loading. RaiSim is expected to include OpenUSD on every supported
architecture.

Binary
======
Installed executable when available: ``nvidia_usd_robots``.

Assets
======
The bundled selectable assets are:

- ``create3``: ``rsc/isaac/Robots/iRobot/Create3/create_3.usd`` (BSD-3)
- ``jetbot``: ``rsc/isaac/Robots/NVIDIA/Robomaker/aws_robomaker_jetbot.usd`` (MIT)
- ``ant``: ``rsc/isaac/Robots/IsaacSim/Ant/ant.usd`` (MIT)

Run
===
List available assets:

.. code-block:: bash

   <raisim-install>/bin/nvidia_usd_robots --list-assets

Run one asset:

.. code-block:: bash

   <raisim-install>/bin/nvidia_usd_robots --asset create3

On Windows, run ``nvidia_usd_robots.exe`` instead. This example uses
RaisimServer. Start ``rayrai_raisim_tcp_viewer`` and connect to port 8080.

For non-visual load tests:

.. code-block:: bash

   <raisim-install>/bin/nvidia_usd_robots --asset create3 --headless-test
   <raisim-install>/bin/nvidia_usd_robots --asset jetbot --headless-test
   <raisim-install>/bin/nvidia_usd_robots --asset ant --headless-test

Details
=======
- Loads the selected USD file with the ``World`` constructor.
- Fails fast if the USD scene does not import as an articulated system or if no
  supported collision bodies are imported.
- Places the imported floating base above the ground and applies a shared
  ``nvidia_usd_robot`` collision material.
