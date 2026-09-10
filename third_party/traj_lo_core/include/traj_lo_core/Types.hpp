// SPDX-License-Identifier: GPL-3.0-only
// mola_traj_lo_wrapper: original code (not derived from Traj-LO), the plain
// data types used at the traj_lo_core boundary. See THIRD_PARTY_NOTICES.md at
// the repository root for the licensing situation of the vendored Traj-LO /
// Sophus / robin-map code this library wraps.
#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace traj_lo_core
{
/** One input LiDAR point. `t_offset_sec` is the point's time relative to the
 * scan's start time (`t_beg`, passed to `TrajLoCore::processScan()`), in
 * seconds and always >= 0: the same convention every other wrapper in this
 * benchmark suite uses.
 *
 * Traj-LO itself is natively integer-timed: internally it works in `int64_t`
 * nanoseconds, and cuts a sweep into segments on nanosecond boundaries. The
 * conversion from these `double` seconds to that integer clock happens in one
 * place only, `TrajLoCore`, which also subtracts a per-session epoch first
 * (see `TrajLoCore::sessionEpoch()`) so that the nanosecond resolution is not
 * spent on the Unix epoch's leading digits.
 */
struct LidarPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
  double t_offset_sec = 0;
};
using LidarPointVector = std::vector<LidarPoint>;

/** One output (local map) point: XYZI only, no per-point timing. */
struct XYZIPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
};
using XYZIPointVector = std::vector<XYZIPoint>;

/** Algorithm parameters. Field names mirror Traj-LO's own `data/config_*.yaml`
 * schema (upstream's `trajectory:` and `mapping:` blocks, plus `calibration:`'s
 * `time_offset`) so a pipeline YAML author can cross-reference upstream docs.
 *
 * Upstream's `dataset:`, `vis:` and the `T_body_lidar` / `T_body_gt` extrinsics
 * are absent on purpose: the host framework owns dataset I/O, visualization and
 * frame conventions. Traj-LO estimates the pose of the LiDAR frame itself.
 */
struct Config
{
  // --- calibration ---
  /// Added to every per-point time before segmentation [s]. Upstream applies
  /// the same correction in its own bag readers.
  double time_offset = 0.0;

  // --- trajectory (upstream `trajectory:`) ---
  /// Length of the very first segment [ns]. Longer than `seg_interval` so the
  /// map is initialized from a full sweep rather than a fraction of one.
  double init_interval = 3e8;
  /// Length of every subsequent sub-scan segment [ns].
  double seg_interval = 4e7;
  /// Number of segment-boundary control poses in the sliding window.
  int seg_num = 3;
  /// Weight of the kinematic (constant-twist) constraint between consecutive
  /// segments, scaled by the segment's inlier count.
  float kinematic_constrain = 2.0f;
  /// Prior weight pinning the very first control pose.
  double init_pose_weight = 1e9;
  /// Gauss-Newton stopping threshold on the maximum absolute increment.
  double converge_thresh = 0.001;
  /// Gauss-Newton iteration cap per segment.
  int max_iterations = 25;

  // --- mapping (upstream `mapping:`) ---
  /// Voxel size of the two-stage input downsampling [m]. Points are reduced at
  /// `ds_size * 0.5` for map insertion and again at `ds_size * 1.5` for
  /// registration.
  float ds_size = 0.4f;
  /// Voxel size of the local map [m].
  float voxel_size = 0.4f;
  /// Maximum number of points kept per map voxel.
  int max_voxel_num = 20;
  /// Planarity gate on the 5-nearest-neighbor plane fit; smaller is stricter.
  float planer_thresh = 0.1f;
  /// Map radius around the current pose [m]; also scales the adaptive
  /// registration threshold.
  float max_range = 120.0f;
  /// Near-field cut [m]. On handheld/backpack rigs the carrier itself sits
  /// inside the first metre and acts as a false zero-motion constraint.
  float min_range = 0.3f;
};

/** Per-segment diagnostics: everything the estimator already computed for the
 * segment whose control pose is being reported. This is the research payload
 * for a LiDAR-only method, where the interesting question is how geometry
 * alone constrains the trajectory over time.
 */
struct Analytics
{
  /// Sum of point-to-plane residuals over the segment's correspondences.
  double last_error = 0;
  /// Number of point-to-plane correspondences found for the segment.
  double last_inliers = 0;
  /// Gauss-Newton iterations actually run (<= `Config::max_iterations`).
  int iterations = 0;
  /// Whether `Config::converge_thresh` was reached before the iteration cap.
  bool converged = false;
  /// Occupied voxels in the local map.
  std::size_t map_voxels = 0;
  /// Segments currently inside the sliding window.
  std::size_t window_size = 0;
  /// Current adaptive point-registration threshold [m].
  double reg_threshold = 0;
  /// Linear velocity implied by the segment's two control poses [m/s],
  /// expressed in the segment's begin-pose frame. Traj-LO carries no explicit
  /// velocity state; this is its closest analogue.
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  /// Angular velocity implied by the same pair [rad/s], same frame.
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  /// Dimension of the marginalization prior carried into the next window.
  std::size_t marg_prior_dim = 6;
};

/** One estimated control pose. Traj-LO has no pose-per-scan: it emits a pose
 * per segment boundary as that boundary leaves the sliding window, so these
 * arrive at roughly `Config::seg_interval` rather than at the scan rate, and a
 * scan may produce several or none.
 */
struct OdometryOutput
{
  /// Absolute time [s], on the same clock as the input scan timestamps.
  double timestamp = 0;
  /// World-from-LiDAR pose at `timestamp`.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Analytics analytics;
};

}  // namespace traj_lo_core
