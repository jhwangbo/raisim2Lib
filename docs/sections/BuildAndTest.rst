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
   * - ``RAISIM_MUJOCO_BENCHMARK``
     - Build the MuJoCo counterparts of the benchmark scenarios so the runner
       can time both engines side by side. Defaults to ``ON``; turn it off when
       the MuJoCo dependency is unavailable.
   * - ``RAISIM_DOC``
     - Build documentation through CMake.
   * - ``RAISIM_ENGINE2``
     - Build the source-tree Engine 2 world authoring layer. Enabled by
       default in the current source tree.
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
      -DRAISIM_MUJOCO_BENCHMARK=ON \
      -DRAISIM_TEST=OFF
    cmake --build build-benchmark --target benchmarks -j12

Drop ``-DRAISIM_MUJOCO_BENCHMARK=ON`` (or set it to ``OFF``) to build only the
RaiSim variants. With MuJoCo enabled the same scenarios are also compiled
against MuJoCo so the runner can produce paired timings in a single report.

Run benchmark timing on one thread:

.. code-block:: bash

    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-benchmark/benchmark/benchmarks --repeat 3
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-benchmark/benchmark/benchmarks --bench granular_dense_contact

Pick which engines run with ``--backend``:

.. code-block:: bash

    ./build-benchmark/benchmark/benchmarks --all --backend=both --report   # default; writes JSON/CSV/Markdown
    ./build-benchmark/benchmark/benchmarks --all --raisim                  # RaiSim only
    ./build-benchmark/benchmark/benchmarks --all --mujoco                  # MuJoCo only

``--report`` writes ``benchmark/report/benchmark_<timestamp>.{json,csv}`` and
refreshes ``benchmark/report.md``. The ``--backend=both`` combination is what
the in-repo ``tools/repo_status_report.py`` consumes when it renders the
RaiSim-vs-MuJoCo comparison chart in ``repo_status.html``.

Use ``-- --help`` after the benchmark selection to inspect benchmark-specific
flags:

.. code-block:: bash

    ./build-benchmark/benchmark/benchmarks --bench granular_dense_contact -- --help

Use :doc:`Performance` for scene-level tuning guidance and for choosing a
representative benchmark before changing collision geometry, solver parameters,
or sensor workloads.

Rayrai checks and benchmarks
============================

Rayrai targets are part of the source build when rayrai dependencies are
available. The most useful verification targets are:

.. code-block:: bash

    cmake --build build-release --target rayrai_check_showcase -j12
    cmake --build build-release --target rayrai_benchmark rayrai_complete_showcase_benchmark rayrai_pbr_first_draw_benchmark -j12
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-release/rayrai/rayrai_benchmark --json
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-release/rayrai/rayrai_complete_showcase_benchmark --json
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-release/rayrai/rayrai_pbr_first_draw_benchmark --json
    cmake --build build-release --target rayrai_benchmark_json -j12
    cmake --build build-release --target rayrai_benchmark_thresholds_test -j12
    cmake --build build-release --target rayrai_quality_metrics_test -j12

The Python-backed rayrai targets are present when CMake finds ``Python3``.

Engine 2 checks and benchmarks
==============================

Engine 2 targets are built when ``RAISIM_ENGINE2=ON``. The command-line tool
output is named ``raisim_engine2`` even though the CMake executable target is
``raisim_engine2_cli``:

.. code-block:: bash

    cmake --build build-release --target raisim_engine2_cli raisim_engine2_headless -j12
    ctest --test-dir build-release -R '^raisim_engine2_' -j12 --output-on-failure
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./build-release/raisim_engine2/raisim_engine2_benchmark

When rayrai and GUI dependencies are available, ``raisim_engine2_app`` builds the
``raisim_engine2_editor`` executable.

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

The docs read the RaiSim version from the source CMake project. Some
source-reference pages are intentionally excluded through ``docs/conf.py`` when
they describe source-only or benchmark workflows rather than published package
pages.
