################################
Map Example: Mountain1 Heightmap
################################

Overview
========
Loads the mountain1 heightmap from the RaisimUnreal map assets and drops an Aliengo robot on it with PD control. It sets the RaisimServer map to "mountain1" so the visualizer matches the terrain.

Screenshot
==========
.. image:: ../../../image/map_mountain1.png

Binary
======
Installed executable: ``map_mountain1_heightmap``.

Run
====
Run the installed executable:

.. code-block:: bash

   <raisim-install>/bin/map_mountain1_heightmap

On Windows, run ``map_mountain1_heightmap.exe`` instead.
This example uses RaisimServer. Start a visualizer client (RaisimUnity, RaisimUnreal, or the rayrai TCP viewer) and connect to port 8080.


Details
=======
- Loads the mountain1 heightmap PNG with scale/offset and hidden mesh.
- Spawns Aliengo with PD posture control.
- Sets the Unreal map to ``mountain1`` and focuses on the robot.

