############################
Map Example: Hill1 Heightmap
############################

Overview
========
Loads the hill1 heightmap and drops Aliengo high above the terrain to demonstrate heightmap placement and scale. The map is set to "hill1" in the server.

Screenshot
==========
.. image:: ../../../image/map_hill1.png

Source Status
=============
Source file: ``examples/src/maps/map_hill1_heightmap.cpp``.

This page is excluded from the published docs, and the current examples CMake
file does not register this source as an installed executable. Treat it as a
source reference unless you register it in a local examples build.

For visualization, use ``rayrai_raisim_tcp_viewer`` with RaisimServer-based
applications.

Details
=======
- Loads the hill1 heightmap PNG with explicit scale/offset and hides the mesh.
- Drops Aliengo from height and holds posture with PD gains.
- Sets the server map name to ``hill1`` and focuses the camera on the robot.

