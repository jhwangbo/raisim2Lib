############################
Map Example: Lake1 Heightmap
############################

Overview
========
Uses the lake1 heightmap image and spawns Aliengo to walk on the terrain. This example shows how to configure heightmap scaling and map selection for a large outdoor scene.

Screenshot
==========
.. image:: ../../../image/map_lake1.png

Binary
======
Installed executable: ``map_lake1_heightmap``.

Run
====
Run the installed executable:

.. code-block:: bash

   <raisim-install>/bin/map_lake1_heightmap

On Windows, run ``map_lake1_heightmap.exe`` instead.
This example uses RaisimServer. Start a visualizer client (RaisimUnity, RaisimUnreal, or the rayrai TCP viewer) and connect to port 8080.


Details
=======
- Loads the lake1 heightmap PNG with scale/offset.
- Spawns Aliengo with PD posture control at a low start height.
- Sets the Unreal map to ``lake1`` and focuses on the robot.

