/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: wraps the MIT-licensed Traj-LO algorithm
 (third_party/traj_lo_core), copyright Xin Zheng.
*/

/**
 * @file   TrajLoOdometry.h
 * @brief  MOLA front-end module wrapping the Traj-LO algorithm
 */
#pragma once

#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/viz/CSetOfLines.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <traj_lo_core/Types.hpp>

namespace traj_lo_core
{
class TrajLoCore;
}

namespace mola
{
/** MOLA online front-end wrapping Traj-LO, a continuous-time, **LiDAR-only**
 * odometry method (see `traj_lo_core`, in `third_party/`).
 *
 * Consumes `mrpt::obs::CObservationPointCloud` observations and publishes live
 * localization + map updates via `LocalizationSourceBase`/`MapSourceBase`. It
 * takes no IMU: that is the point of the method, and of including it in this
 * benchmark suite as the zero-inertial-authority control arm.
 *
 * Two things differ from the other wrappers in the suite and are worth knowing
 * before reading a trajectory it produced:
 *
 * - **There is no pose per scan.** Traj-LO cuts each sweep into segments of
 *   `seg_interval` and estimates the control poses at the segment boundaries
 *   jointly over a sliding window, so poses come out at the segment rate
 *   (~25 Hz at the default 40 ms) rather than the scan rate, and a given scan
 *   may yield several or none.
 * - **A pose is reported only once it leaves the sliding window**, i.e. after
 *   it has been refined and then marginalized. The trajectory drawn live in
 *   the GUI therefore lags the newest scan by about `seg_num * seg_interval`.
 *
 * Scans are processed by Traj-LO's own bounded queue and processing thread,
 * which are part of the method's design and are kept; this module does not add
 * a worker pool of its own. The offline CLI (`mola-traj-lo-cli`) achieves its
 * loss-free guarantee by feeding one scan at a time and busy-waiting on
 * `isBusy()`.
 *
 * \ingroup mola_traj_lo_wrapper_grp
 */
class TrajLoOdometry : public FrontEndBase, public LocalizationSourceBase, public MapSourceBase
{
  DEFINE_MRPT_OBJECT(TrajLoOdometry, mola)

public:
  TrajLoOdometry();
  ~TrajLoOdometry() override;

  // ExecutableBase
  void spinOnce() override;
  void onQuit() override;

  // RawDataConsumer
  void onNewObservation(const mrpt::obs::CObservation::ConstPtr & o) override;

  /** True while any scan fed so far is still queued or being processed. Polled
   * by `mola-traj-lo-cli` to guarantee no scan is ever dropped. */
  bool isBusy() const;

  /** Stops the estimator, which first emits the control poses still inside the
   * sliding window, and publishes them. Without this the trajectory would be
   * missing its last `seg_num * seg_interval` of poses. Called automatically
   * on shutdown; the offline CLI calls it before saving. Idempotent. */
  void finish();

  /** Trajectory accumulated so far (`map` -> `lidar`). */
  mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

protected:
  void initialize_frontend(const Yaml & cfg) override;

private:
  void onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void drainAndPublish();
  void publishOutput(const traj_lo_core::OdometryOutput & out);
  void shutdownCleanup();

  // --- GUI / 3D scene visualization (mola_viz / mola_viz_imgui) ---
  void updateVisualization(
    const mrpt::poses::CPose3D & lidarPose, const traj_lo_core::OdometryOutput & out);
  void updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose);
  void updateVisualizationMap();
  void internalBuildGUI();
  void updateVisualizationTextLabels(const traj_lo_core::OdometryOutput & out);

  std::regex lidar_sensor_label_regex_{"lidar"};

  /// Pose of the LiDAR frame wrt base_link ("x y z yaw_deg pitch_deg
  /// roll_deg"), i.e. `T_base_lidar`. Traj-LO estimates the LiDAR frame's own
  /// pose; when this is not identity, the reported trajectory is
  /// right-composed with its inverse so that it is `map -> base_link` instead.
  /// That matters for datasets whose ground truth is given in a base frame
  /// that differs from the raw LiDAR frame (Oxford Spires: a 180 deg yaw and a
  /// 0.124 m lever arm). An aligned APE barely notices the difference, but it
  /// negates the horizontal part of every relative motion, so relative drift
  /// and tilt metrics would measure the frame convention instead of the
  /// odometry. Identity by default.
  mrpt::poses::CPose3D lidar_pose_in_baselink_;
  mrpt::poses::CPose3D lidar_pose_in_baselink_inv_;
  bool report_in_base_frame_ = false;

  /// Used only as a fallback when an incoming point cloud has no per-point
  /// timestamp channel (synthesizes one from the azimuth angle).
  double fallback_scan_period_ = 0.1;

  traj_lo_core::Config core_cfg_;
  std::unique_ptr<traj_lo_core::TrajLoCore> core_;

  mutable std::mutex trajectory_mtx_;
  mrpt::poses::CPose3DInterpolator trajectory_;

  /// Serializes drainAndPublish(). Poses are drained both from the thread that
  /// feeds observations and from this module's own spinOnce(), and publishing
  /// touches state that is not otherwise locked (the 3-D path, the last drawn
  /// map snapshot).
  std::mutex publish_mtx_;

  std::atomic_bool shutdown_cleanup_done_{false};
  std::atomic_bool map_snapshots_enabled_{false};

  /// Logged once, on the first scan: which of the two per-point timing paths
  /// the adapter took. See TrajLoOdometry_Adapters.cpp.
  bool logged_time_source_ = false;

  /// If true, `shutdownCleanup()` dumps `trajectory_` to
  /// `trajectory_output_file_` in TUM format, so that `mola-cli` (online mode)
  /// can export a trajectory the same way the offline CLI does.
  bool save_trajectory_to_file_ = false;
  std::string trajectory_output_file_;

  struct VisualizationParams
  {
    std::atomic_bool show_trajectory{true};
    std::atomic_bool show_map{true};
    std::atomic_bool camera_follows_vehicle{true};

    float current_pose_corner_size = 1.0f;
    float map_point_size = 2.0f;

    /// How many emitted control poses to wait between local-map refreshes.
    int map_update_decimation = 25;
    /// Keep one map point out of every N, for drawing only.
    int map_point_decimation = 4;
  } visualization_params_;

  mrpt::viz::CSetOfLines::Ptr gl_estimated_path_;
  /// The last map snapshot handed to the 3-D scene; the estimator refreshes
  /// the snapshot far less often than poses come out, so redrawing is skipped
  /// while it is still the same one.
  std::shared_ptr<const traj_lo_core::XYZIPointVector> last_drawn_map_;
  bool gui_created_ = false;

  struct Gui
  {
    mola::gui::LiveString::Ptr lbStatus;
  } gui_;
};

}  // namespace mola
