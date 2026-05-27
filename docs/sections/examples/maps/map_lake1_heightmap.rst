############################
Map Example: Lake1 Heightmap
############################

Overview
========
Uses the lake1 heightmap image and spawns Aliengo to walk on the terrain. This example shows how to configure heightmap scaling and map selection for a large outdoor scene.

Screenshot
==========
.. image:: ../../../image/map_lake1.png

Source Status
=============
Source file: ``examples/src/maps/map_lake1_heightmap.cpp``.

This page is excluded from the published docs, and the current examples CMake
file does not register this source as an installed executable. Treat it as a
source reference unless you register it in a local examples build.

For visualization, use ``rayrai_raisim_tcp_viewer`` with RaisimServer-based
applications.

Details
=======
- Loads the lake1 heightmap PNG with scale/offset.
- Spawns Aliengo with PD posture control at a low start height.
- Sets the server map name to ``lake1`` and focuses on the robot.

