#############################
Build, Test, and Benchmark
#############################

This page describes the current source-tree workflow. It is separate from
:doc:`Installation`, which covers installed package paths and activation.
For repository layout and build output paths, see :doc:`ProjectLayout`.

Common CMake options
====================

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Option
     - Meaning
   * - ``RAISIM_EXAMPLE``
     - Build C++ examples.
   * - ``RAISIM_TEST``
     - Build unit tests. This is enabled by default in the current source tree.
   * - ``RAISIM_BENCHMARK``
     - Build benchmark executables.
   * - ``RAISIM_DOC``
     - Build documentation through CMake.
   * - ``RAISIM_FETCH_EIGEN``
     - Fetch Eigen through CMake instead of relying on a system package.
   * - ``RAISIM_LICENSE_MANAGER``
     - Build the license manager target.

Build examples and tests
========================

.. code-block:: bash

    cd /path/to/raisim
    cmake -S . -B build-release \
      -DCMAKE_BUILD_TYPE=Release \
      -DRAISIM_EXAMPLE=ON \
      -DRAISIM_TEST=ON
    cmake --build build-release -j12

Run tests with 12 workers:

.. code-block:: bash

    ctest --test-dir build-release -j12 --output-on-failure

Build benchmarks
================

Configure benchmarks explicitly:

.. code-block:: bash

    cmake -S . -B build-benchmark \
      -DCMAKE_BUILD_TYPE=Release \
      -DRAISIM_BENCHMARK=ON \
      -DRAISIM_TEST=OFF
    cmake --build build-benchmark --target benchmarks -j12

Run benchmark timing on one thread:

.. code-block:: bash

    ./build-benchmark/bin/benchmarks --repeat 3
    ./build-benchmark/bin/benchmarks --bench granular_dense_contact

Use ``-- --help`` after the benchmark selection to inspect benchmark-specific
flags:

.. code-block:: bash

    ./build-benchmark/bin/benchmarks --bench granular_dense_contact -- --help

Use :doc:`Performance` for scene-level tuning guidance and for choosing a
representative benchmark before changing collision geometry, solver parameters,
or sensor workloads.

Rayrai checks and benchmarks
============================

Rayrai targets are part of the source build when rayrai dependencies are
available. The most useful verification targets are:

.. code-block:: bash

    cmake --build build-release --target rayrai_check_showcase -j12
    ./build-release/bin/rayrai_benchmark --json

Build the documentation
=======================

Prefer the CMake documentation target when you need API reference blocks,
because CMake generates Doxygen XML and passes the Breathe project path to
Sphinx:

.. code-block:: bash

    cmake -S $HOME/raisim2Lib/docs -B $HOME/raisim2Lib/build-docs \
      -DRAISIM_DOCS_RAISIM_INCLUDE_DIR=$HOME/raisim2Lib/raisim/include/raisim \
      -DRAISIM_DOCS_RAYRAI_INCLUDE_DIR=$HOME/raisim2Lib/rayrai/include/rayrai
    cmake --build $HOME/raisim2Lib/build-docs -j12

For prose and link checks, the documentation can also be built directly with
Sphinx:

.. code-block:: bash

    $HOME/raisim2Lib/docs/.venv/bin/sphinx-build \
      -b html $HOME/raisim2Lib/docs $HOME/raisim2Lib/docs/_build/html

Direct Sphinx builds do not generate Doxygen XML by themselves. They are useful
for quick RST validation, but Breathe API directives will warn unless
``breathe_projects.raisim`` points to generated Doxygen XML.
See :doc:`Troubleshooting` for the common warning modes.

The current docs read the RaiSim version from the source CMake project and
exclude legacy generated example pages whose source files no longer exist in the
compact current example layout.
