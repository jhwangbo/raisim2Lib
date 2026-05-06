#######################
Map Example: Kinova Arm
#######################

Overview
========
Loads the Kinova arm, applies joint PD gains and targets, and runs it on the simple map. This example focuses on manipulator setup and joint-level control.

Binary
======
Installed executable: ``map_kinova_arm``.

Run
====
Run the installed executable:

.. code-block:: bash

   <raisim-install>/bin/map_kinova_arm

On Windows, run ``map_kinova_arm.exe`` instead.
This example uses RaisimServer. Start a visualizer client (RaisimUnity, RaisimUnreal, or the rayrai TCP viewer) and connect to port 8080.


Details
=======
- Loads the Kinova arm URDF on flat ground and applies joint PD gains/targets.
- Uses the ``simple`` Unreal map for fast rendering.
- Focuses the camera on the arm for a fixed-base articulation demo.

