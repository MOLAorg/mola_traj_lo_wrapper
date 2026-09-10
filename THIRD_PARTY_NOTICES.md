# Third-party notices

This package (`mola_traj_lo_wrapper`) is released under **GPL-3.0**. It bundles,
under `third_party/`, code from the projects listed below. All of them are
permissively licensed (MIT), which is compatible with GPL-3.0: the combined work
is distributed under GPL-3.0 while each bundled file keeps its own copyright
notice and license text.

---

## Traj-LO

- **Upstream:** https://github.com/kevin2431/Traj-LO
- **Commit copied:** `ba273d36d4b69a557eedd19ba80e1c90ab30039d` (2024-07-13)
- **License:** MIT (see `third_party/traj_lo_core/upstream/LICENSE`)
- **Copyright:** Copyright (c) 2023 Xin Zheng \<xinzheng@zju.edu.cn\>
- **Paper:** X. Zheng and J. Zhu, *Traj-LO: In Defense of LiDAR-Only Odometry
  Using an Effective Continuous-Time Trajectory*, IEEE RA-L, 2024.
- **Location:** `third_party/traj_lo_core/upstream/`

Parts of Traj-LO's `map_manager` (`VoxelBlock`, `VoxelHash`,
`UpdateModelDeviation`, `ComputeModelError`, `ComputeThreshold`) are themselves
derived from **KISS-ICP** (https://github.com/PRBonn/kiss-icp), MIT, Copyright
(c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
Their notice is preserved at the top of `map_manager.h`.

### What was copied

Only the estimator: `src/core/`, and `include/trajlo/{core,utils}/`.

### What was dropped

- `src/io/` and `include/trajlo/io/` -- upstream's own vendored ROS 1 bag
  readers. MOLA's dataset sources replace them.
- `src/gui/` and `include/trajlo/gui/` -- upstream's OpenGL/imgui visualizer,
  and with it the `imgui`, `implot`, `glad`, `glm` and GLFW dependencies. MOLA's
  `mola_viz` replaces it.
- `run_trajlo.cpp` -- replaced by `traj_lo_core::TrajLoCore` plus the MOLA
  module and CLI.
- `src/utils/config.cpp` -- the yaml-cpp loader for `TrajConfig`. MOLA owns
  configuration and fills the struct field by field from the pipeline YAML, so
  the loader had no caller and the package needs no yaml-cpp.
- `thirdparty/{eigen,yaml-cpp,oneTBB}` -- system packages are used instead
  (Eigen 3.4, TBB 2022.x). `thirdparty/{Sophus,robin-map}` are vendored here
  instead of as submodules, see below.

### What was changed

Everything below is marked in the sources with a `MOLA port` comment.

1. **A per-pose output callback** (`odometry.h`, `odometry.cpp`). Upstream only
   appended to an internal `trajectory_` and wrote it to a file at the end.
   `TrajLOdometry::pose_callback` is invoked at the same three places that
   append to `trajectory_` -- the standing-start initialization, each
   marginalization, and the final flush of the sliding window -- so poses can be
   consumed as they are produced. `trajectory_` itself is untouched.
2. **Per-segment diagnostics** (`SegmentDiagnostics` in `odometry.h`). The
   callback also carries the segment's residual, inlier count, Gauss-Newton
   iteration count and convergence flag, the map's voxel count, the sliding
   window's size, the adaptive registration threshold, and the twist implied by
   the segment's two control poses. Everything there was already computed;
   `last_iterations_` / `last_converged_` are the only new state.
3. **An unconsumed-scan counter** (`TrajLOdometry::pending_scans`). Needed for
   the loss-free offline path: a scan is off the input queue while it is being
   processed, so the queue's size alone cannot say whether the estimator is
   idle.
4. **A map snapshot accessor** (`TrajLOdometry::MapSnapshot`,
   `MapManager::Snapshot`/`MapSize`/`RegThreshold`), taken on the processing
   thread so the map needs no lock. Replaces upstream's visualizer feed.
5. **Deterministic reduction** (`map_manager.cpp`). `tbb::parallel_reduce`
   splits and joins its partial floating-point sums in an order that depends on
   task stealing, so the same input did not give the same trajectory twice.
   Under the `TRAJLO_DETERMINISTIC` CMake option, on by default,
   `tbb::parallel_deterministic_reduce` is used instead. Turning the option off
   restores upstream's behavior exactly.
6. **A data race in the point-registration loop, fixed** (`map_manager.cpp`).
   Upstream evaluated "is either control pose already linearized?" *inside* the
   parallel loop and, when it was, overwrote four enclosing-scope variables
   (`tangent`, `R_w_b_t`, `R_w_e_t`, `R_e_b`) from every worker thread. Besides
   being undefined behavior, it made the result depend on how far each thread
   had progressed before another overwrote them. None of those quantities
   depends on the point index, so they are hoisted out of the loop and evaluated
   once. The residual and its correspondence search keep using the current (not
   linearized) pose exactly as upstream does; only the Jacobian is taken at the
   fixed linearization point, which is what the branch was for. This changes
   results marginally with respect to a single-threaded upstream run.
7. **Iterator invalidation while pruning the map, fixed** (`map_manager.cpp`,
   `MapManager::Update`). Upstream erased from the `tsl::robin_map` while
   iterating it with a range-`for`. That container's erase moves later elements
   to fill the hole, so voxels were skipped and the traversal was undefined
   behavior. The keys are now collected first and erased afterwards; the set of
   voxels removed is the one upstream meant to remove.
8. **A 0/0 in the adaptive threshold, fixed** (`map_manager.h`,
   `MapManager::ComputeThreshold`). Upstream fell through to
   `sqrt(model_error_sse2_ / num_samples_)` even with `num_samples_ == 0`,
   yielding a NaN threshold that rejects every correspondence for the rest of
   the run. The early exit of the KISS-ICP code the block is derived from is
   restored.
9. **`TrajConfig` trimmed** (`config.h`). The `dataset:` (data type, topic,
   path, pose saving), `vis:` and `T_body_lidar`/`T_body_gt` fields are gone
   along with the yaml-cpp loader; the remaining fields carry upstream's
   `data/config_hesai.yaml` values as in-class defaults. The corresponding dead
   blocks in `odometry.cpp` (pose saving, the visualizer feed, and
   `UndistortRawPoints`, whose only caller was that feed) were removed.
10. **A null-thread guard in the destructor** (`odometry.h`). Upstream joined
    `processing_thread_` unconditionally, which crashes if `Start()` was never
    called.

Vendored upstream code is deliberately **not** reformatted: it keeps its
original style so that diffs against upstream stay readable. Only the code
written for this package (the module, the app, `traj_lo_core`'s own boundary and
tests) follows this repository's `.clang-format`.

---

## Sophus

- **Upstream:** https://github.com/strasdat/Sophus
- **Commit copied:** `de0f8d3d92bf776271e16de56d1803940ebccab9` (tag `1.22.10`)
- **License:** MIT (see `third_party/Sophus/LICENSE.txt`)
- **Copyright:** Copyright (c) 2011-2017 Hauke Strasdat, 2012-2017 Steven Lovegrove
- **Location:** `third_party/Sophus/sophus/`
- **Changes:** none. Header-only, copied verbatim; only the Ceres bindings
  (`ceres_local_parameterization.hpp`, `ceres_manifold.hpp`,
  `ceres_typetraits.hpp`) and `test_macros.hpp` were left out, since nothing
  here uses Ceres. Built with `SOPHUS_USE_BASIC_LOGGING`, as upstream Traj-LO
  does, so no `fmt` dependency is pulled in.

Vendored because there is no `libsophus-dev` on the target platform.

---

## robin-map (tsl::robin_map)

- **Upstream:** https://github.com/Tessil/robin-map
- **Commit copied:** `4ec1bf19c6a96125ea22062f38c2cf5b958e448e` (tag `v1.4.0`)
- **License:** MIT (see `third_party/robin-map/LICENSE`)
- **Copyright:** Copyright (c) 2017 Thibaut Goetghebuer-Planchon
- **Location:** `third_party/robin-map/include/tsl/`
- **Changes:** none. Header-only, copied verbatim.

Vendored because there is no `libtsl-robin-map-dev` on the target platform.
Note MRPT and `mp2p_icp` vendor their own copies of the same library; this one
is kept private to `traj_lo_core` (a `PRIVATE` include directory), so the two
never meet in one translation unit.

---

## License compatibility

Traj-LO, Sophus and robin-map are all MIT, which imposes only attribution.
Combining them into this GPL-3.0 package is permitted, and this file plus the
bundled `LICENSE` files satisfy the attribution requirement. There is no
GPL-2.0-only component here, so this package has none of the licensing
incompatibility that affects some other wrappers in this suite.
