rayrai_benchmark
================

``rayrai_benchmark`` measures rayrai rendering and serialization paths, including
shadowed and unshadowed color rendering, readback, depth, picking, transparent
visuals, culling, instanced visuals, point clouds, quality presets, HDR/IBL
setup, scene synchronization, and TCP deformable serialization.

Run:

.. code-block:: bash


   <raisim-install>/bin/rayrai_benchmark

When Python support is available, the ``rayrai_benchmark_json`` custom target
can run the benchmark and write the configured JSON path.
