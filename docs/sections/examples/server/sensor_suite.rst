############################
Server Example: Sensor Suite
############################

Overview
========
Demonstrates camera, depth, IMU, and LiDAR sensors on ANYmal, including depth-to-point cloud conversion and point cloud visualization. It is the main reference for sensor APIs.

Screenshot
==========
.. image:: ../../../image/sensors_cpp.png

Binary
======
Installed executable: ``sensor_suite``.

Run
====
Run the installed executable:

.. code-block:: bash

   <raisim-install>/bin/sensor_suite

On Windows, run ``sensor_suite.exe`` instead.
This example uses RaisimServer. Start a visualizer client (RaisimUnity, RaisimUnreal, or the rayrai TCP viewer) and connect to port 8080.


Details
=======
- Loads ANYmal with RGB, depth, IMU, and LiDAR sensors.
- Uses VISUALIZER measurement source for cameras and converts depth to point clouds.
- Visualizes LiDAR scans via a point cloud (Unreal-only).

