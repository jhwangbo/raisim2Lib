#############################
Project Layout
#############################

The current RaiSim source tree is organized around one top-level CMake project.
This page maps the directories users most often need while building examples,
tests, benchmarks, and rayrai tools.

Source Directories
==================

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Path
     - Purpose
   * - ``include/raisim``
     - Public RaiSim headers installed with the ``raisim`` CMake package.
   * - ``src``
     - RaiSim implementation sources.
   * - ``test``
     - CTest targets for core physics, objects, sensors, import/export, and
       feature-specific behavior.
   * - ``benchmark``
     - Unified benchmark runner and individual benchmark implementations.
   * - ``examples``
     - Current top-level example targets named ``example_*``.
   * - ``visualizer/rayrai``
     - rayrai renderer library, TCP viewer, rendering examples, benchmarks, and
       renderer-focused tests.
   * - ``res`` and ``rsc``
     - Runtime resources such as robot models, meshes, textures, USD/glTF
       assets, and example data.
   * - ``prebuilt/openusd``
     - OpenUSD runtime used by USD import support on supported platforms.
   * - ``third_party``
     - Bundled third-party libraries built as part of the source tree.
   * - ``cmake``
     - CMake helpers and package config templates.

Build Directories
=================

Build directories are not part of the source tree. The docs use these names for
clarity:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Build directory
     - Typical use
   * - ``build-release``
     - Release examples and tests.
   * - ``build-benchmark``
     - Release benchmarks with ``RAISIM_BENCHMARK=ON``.
   * - ``build-debug``
     - Debug build for local debugging.
   * - ``build-docs``
     - CMake-driven docs build that also generates Doxygen XML for Breathe.

Source-tree executables are placed under ``<build-dir>/bin``. For example:

.. code-block:: bash

    ./build-release/bin/example_anymal_contacts
    ./build-release/bin/rayrai_raisim_tcp_viewer
    ./build-benchmark/bin/benchmarks

Some older generated docs and package layouts used paths such as
``build/examples`` or ``build-examples/examples``. For the current source tree,
prefer ``<build-dir>/bin`` unless a specific installed package says otherwise.

Installed Package Layout
========================

A local install creates separate package prefixes for RaiSim and rayrai:

.. code-block:: text

    $RAISIM_LOCAL_INSTALL_ROOT/raisim
    $RAISIM_LOCAL_INSTALL_ROOT/rayrai

Downstream projects use ``CMAKE_PREFIX_PATH`` to find these installed packages.
See :doc:`Installation` for environment setup and activation.

Where To Add New Things
=======================

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Change
     - Usual location
   * - New RaiSim public API
     - Header in ``include/raisim`` and implementation in ``src``.
   * - New physics or importer test
     - ``test`` plus a CTest registration in the existing test CMake flow.
   * - New benchmark case
     - ``benchmark`` and registration in the unified benchmark runner.
   * - New user-facing example
     - ``examples`` with an ``example_*`` target.
   * - New rayrai renderer feature
     - ``visualizer/rayrai`` with showcase/check image coverage when visual
       output changes.
   * - New docs page
     - ``docs/sections`` and the relevant ``toctree`` in ``index.rst`` or a
       section index page.

Keep examples focused on demonstrating APIs. Use tests for behavioral
guarantees and benchmarks for timing claims.
