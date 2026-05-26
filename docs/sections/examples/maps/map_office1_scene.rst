##########################
Map Example: Office1 Scene
##########################

Overview
========
Loads the office1 XML world, adds a dynamic ball, and spawns Aliengo with PD control. The server map is set to "office1" for the matching office environment.

Screenshot
==========
.. image:: ../../../image/map_office1.png

Source Status
=============
Source file: ``examples/src/maps/map_office1_scene.cpp``.

This page is excluded from the published docs, and the current examples CMake
file does not register this source as an installed executable. Treat it as a
source reference unless you register it in a local examples build.

For visualization, use ``rayrai_raisim_tcp_viewer`` with RaisimServer-based
applications.

Details
=======
- Loads the office1 XML world and adds a moving sphere.
- Spawns Aliengo with PD posture control on top of the scene.
- Uses the ``office1`` map and focuses the camera on the robot.

