#############################
Project Layout
#############################

The current RaiSim source tree is organized around one top-level CMake project.
This page maps the directories users most often need while building examples,
``raisimPy``, documentation, and rayrai tools.

Source Directories
==================

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Path
     - Purpose
   * - ``examples``
     - Current C++ examples, including server examples and rayrai examples.
   * - ``raisim``
     - Installed RaiSim package prefix with headers, libraries, and CMake
       package files.
   * - ``rayrai``
     - Installed rayrai package prefix with headers, libraries, tools, and
       CMake package files.
   * - ``raisimPy``
     - Python wrapper sources built when ``RAISIM_PY=ON``.
   * - ``rsc``
     - Runtime resources such as robot models, meshes, textures, USD/glTF
       assets, and example data.
   * - ``thirdParty``
     - Bundled third-party libraries built as part of the source tree.
   * - ``cmake``
     - CMake helper modules.

Build Directories
=================

Build directories are not part of the source tree. The docs use these names for
clarity:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Build directory
     - Typical use
   * - ``build``
     - Default examples and optional ``raisimPy`` build.
   * - ``build-examples``
     - Examples-only build used in some docs.
   * - ``build-docs``
     - CMake-driven docs build that also generates Doxygen XML for Breathe.

Source-tree examples are placed under ``<build-dir>/examples``. For example:

.. code-block:: bash

    source ./raisim_env.sh
    ./build/examples/primitive_grid
    ./build/examples/rayrai_tcp_viewer

Source ``raisim_env.sh`` before running built examples so both RaiSim and rayrai
shared libraries are on the platform loader path.

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
