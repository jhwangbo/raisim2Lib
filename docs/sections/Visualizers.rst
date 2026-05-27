#############################
Visualizers
#############################

Rayrai is the current supported visualization path for RaiSim. Older
RaisimUnity and RaisimUnreal integrations are no longer shipped as supported
workflows in the current binary distribution; see :doc:`LegacyIntegrations` for
replacement paths when following old examples or links.

Current Workflows
*****************

.. list-table::
   :header-rows: 1
   :widths: 28 36 36

   * - Workflow
     - Use it when
     - Main executable/API
   * - ``RaisimServer`` + rayrai TCP viewer
     - You want to inspect a running simulation from a separate viewer process.
     - ``raisim::RaisimServer`` and ``rayrai_raisim_tcp_viewer``
   * - In-process rayrai
     - Rendering is part of your application, sensor pipeline, screenshot tool,
       benchmark, or custom UI.
     - ``raisin::RayraiWindow``

``RaisimServer`` + rayrai TCP viewer
====================================

Start the simulation server in your application, then launch the viewer binary:

.. code-block:: bash

   <raisim-install>/bin/rayrai_raisim_tcp_viewer

The TCP viewer receives world state from ``RaisimServer`` and renders it with
rayrai. Use it for normal debugging, camera control, collision-body inspection,
object selection, and server-based examples. The viewer is intentionally a
visualization client: it does not write RGB/depth images back into RaiSim
sensors.

In-process rayrai
=================

Construct ``raisin::RayraiWindow`` directly when your code needs render targets,
readback, custom visuals, or UI embedding:

.. code-block:: cpp

   auto world = std::make_shared<raisim::World>();
   raisin::RayraiWindow viewer(world, 1280, 720);
   viewer.update(1280, 720, false, 0, 0, false);
   unsigned int colorTexture = viewer.getImageTexture();

Use this workflow for RGB/depth sensor rendering, screenshots, glTF/USD asset
inspection, PBR material previews, point clouds, picking, offscreen rendering,
and custom ImGui/Qt tools.

Rayrai feature coverage
=======================

The current rayrai renderer includes:

* ``Fast``/``Balanced``/``High``/``Ultra`` render-quality presets.
* PBR materials, glTF/GLB import, OpenUSD visual import, HDR image-based
  lighting, reflection probes, planar reflections, and authored scene lights.
* Weather and scene effects including time-of-day sky, fog/local fog volumes,
  rain, snow, lightning diagnostics, wet/snow material response, projected
  decals, and irradiance volumes.
* RGB/depth sensor alignment, linear depth rendering, GPU LiDAR slices, point
  clouds, coordinate frames, and camera frustums.
* Capture and diagnostics helpers for supersampled screenshots, debug-pass
  captures, render-pass timings, luminance/exposure analysis, and structured
  JSON diagnostics.

See :doc:`Rayrai` for API details and :doc:`Visualization` for the workflow
boundary between the TCP viewer and in-process rendering.
