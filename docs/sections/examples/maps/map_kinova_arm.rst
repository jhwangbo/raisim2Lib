#######################
Map Example: Kinova Arm
#######################

Overview
========
Loads the Kinova arm, applies joint PD gains and targets, and runs it on the simple map. This example focuses on manipulator setup and joint-level control.

Source Status
=============
Source file: ``examples/src/maps/map_kinova_arm.cpp``.

This page is excluded from the published docs, and the current examples CMake
file does not register this source as an installed executable. Treat it as a
source reference unless you register it in a local examples build.

For visualization, use ``rayrai_raisim_tcp_viewer`` with RaisimServer-based
applications.

Details
=======
- Loads the Kinova arm URDF on flat ground and applies joint PD gains/targets.
- Uses the ``simple`` server map for fast rendering.
- Focuses the camera on the arm for a fixed-base articulation demo.

