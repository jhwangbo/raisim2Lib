#########################################
Server Example: Synchronous Server Update
#########################################

Overview
========
Runs the RaisimServer in a manual socket loop and updates sensors only when the client requests them. This shows how to drive synchronous visualization and sensor updates.

Source Status
=============
Source file: ``examples/src/server/synchronous_server_update.cpp``.

This page is excluded from the published docs, and the current examples CMake
file does not register this source as an installed executable. Treat it as a
source reference unless you register it in a local examples build.

For visualization, use ``rayrai_raisim_tcp_viewer`` with RaisimServer-based
applications.

Details
=======
- Runs RaisimServer in synchronous request/response mode.
- Manually accepts a TCP connection and processes sensor update requests.
- Uses VISUALIZER RGB/depth sensors with ``needsSensorUpdate()``.

