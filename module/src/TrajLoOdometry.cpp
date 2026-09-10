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
 * @file   TrajLoOdometry.cpp
 * @brief  MOLA front-end module wrapping the Traj-LO algorithm
 */
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_traj_lo_wrapper/TrajLoOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <traj_lo_core/TrajLoCore.h>

#include <sstream>

#include "TrajLoOdometry_Adapters.h"

namespace mola
{

IMPLEMENTS_MRPT_OBJECT(TrajLoOdometry, FrontEndBase, mola)

namespace
{
/// Parses "x y z yaw_deg pitch_deg roll_deg".
mrpt::poses::CPose3D parsePose6D(const std::string & s)
{
  std::istringstream iss(s);
  double x, y, z, yaw, pitch, roll;
  if (!(iss >> x >> y >> z >> yaw >> pitch >> roll)) {
    THROW_EXCEPTION_FMT(
      "Could not parse pose string '%s' (expected: 'x y z yaw_deg pitch_deg roll_deg')", s.c_str());
  }
  return {x, y, z, mrpt::DEG2RAD(yaw), mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll)};
}

mrpt::poses::CPose3D isometryToPose(const Eigen::Isometry3d & iso)
{
  mrpt::math::CMatrixDouble44 m;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      m(r, c) = iso.matrix()(r, c);
    }
  }
  return mrpt::poses::CPose3D(m);
}
}  // namespace

TrajLoOdometry::TrajLoOdometry() = default;
TrajLoOdometry::~TrajLoOdometry() { shutdownCleanup(); }

void TrajLoOdometry::onQuit() { shutdownCleanup(); }

void TrajLoOdometry::finish()
{
  if (!core_) {
    return;
  }
  core_->finish();
  drainAndPublish();
}

void TrajLoOdometry::shutdownCleanup()
{
  if (shutdown_cleanup_done_.exchange(true)) {
    return;
  }

  finish();

  if (save_trajectory_to_file_ && !trajectory_output_file_.empty()) {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    MRPT_LOG_INFO_STREAM(
      "Saving estimated trajectory with " << trajectory_.size() << " control poses to file '"
                                          << trajectory_output_file_ << "' in TUM format...");
    trajectory_.saveToTextFile_TUM(trajectory_output_file_);
    MRPT_LOG_INFO("Final trajectory saved.");
  }
}

void TrajLoOdometry::initialize_frontend(const Yaml & cfg)
{
  MRPT_TRY_START

  std::string lidar_sensor_label = "lidar";
  YAML_LOAD_OPT(lidar_sensor_label, std::string);
  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);

  // Loaded into non-underscore locals first, then assigned to the
  // (trailing-underscore) members: YAML_LOAD_OPT's key is the literal C++
  // identifier text, so passing the member name directly would look for a
  // YAML key with a trailing underscore too, which pipeline YAMLs don't use.
  std::string baselink2lidar_pose_str = "0 0 0 0 0 0";
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);
  lidar_pose_in_baselink_ = parsePose6D(baselink2lidar_pose_str);
  lidar_pose_in_baselink_inv_ = -lidar_pose_in_baselink_;
  report_in_base_frame_ = lidar_pose_in_baselink_ != mrpt::poses::CPose3D::Identity();

  double fallback_scan_period = fallback_scan_period_;
  YAML_LOAD_OPT(fallback_scan_period, double);
  fallback_scan_period_ = fallback_scan_period;

  YAML_LOAD_OPT3(core_cfg_, time_offset, double);

  YAML_LOAD_OPT3(core_cfg_, init_interval, double);
  YAML_LOAD_OPT3(core_cfg_, seg_interval, double);
  YAML_LOAD_OPT3(core_cfg_, seg_num, int);
  YAML_LOAD_OPT3(core_cfg_, kinematic_constrain, float);
  YAML_LOAD_OPT3(core_cfg_, init_pose_weight, double);
  YAML_LOAD_OPT3(core_cfg_, converge_thresh, double);
  YAML_LOAD_OPT3(core_cfg_, max_iterations, int);

  YAML_LOAD_OPT3(core_cfg_, ds_size, float);
  YAML_LOAD_OPT3(core_cfg_, voxel_size, float);
  YAML_LOAD_OPT3(core_cfg_, max_voxel_num, int);
  YAML_LOAD_OPT3(core_cfg_, planer_thresh, float);
  YAML_LOAD_OPT3(core_cfg_, max_range, float);
  YAML_LOAD_OPT3(core_cfg_, min_range, float);

  YAML_LOAD_OPT3(visualization_params_, show_trajectory, bool);
  YAML_LOAD_OPT3(visualization_params_, show_map, bool);
  YAML_LOAD_OPT3(visualization_params_, camera_follows_vehicle, bool);
  YAML_LOAD_OPT3(visualization_params_, current_pose_corner_size, float);
  YAML_LOAD_OPT3(visualization_params_, map_point_size, float);
  YAML_LOAD_OPT3(visualization_params_, map_update_decimation, int);
  YAML_LOAD_OPT3(visualization_params_, map_point_decimation, int);

  YAML_LOAD_MEMBER_OPT(save_trajectory_to_file, bool);
  YAML_LOAD_MEMBER_OPT(trajectory_output_file, std::string);

  ASSERT_GT_(core_cfg_.seg_interval, 0.0);
  ASSERT_GT_(core_cfg_.seg_num, 1);

  core_ = std::make_unique<traj_lo_core::TrajLoCore>(core_cfg_);

  MRPT_LOG_INFO_STREAM(
    "TrajLoOdometry initialized (LiDAR-only, no IMU). lidar_sensor_label='"
    << lidar_sensor_label << "' seg_interval=" << core_cfg_.seg_interval * 1e-6
    << " ms seg_num=" << core_cfg_.seg_num << " min_range=" << core_cfg_.min_range
    << " m. Reporting poses of frame: " << (report_in_base_frame_ ? "base_link" : "lidar"));

  MRPT_TRY_END
}

void TrajLoOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  ASSERT_(o);

  // No IMU branch: Traj-LO is LiDAR-only by design. An IMU observation
  // reaching this module is simply not for it.
  if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidarObservation(o);
  }
  MRPT_TRY_END
}

void TrajLoOdometry::onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (!pc || !core_) {
    return;
  }

  // Snapshotting the local map costs a copy per refresh, so it is only turned
  // on once something is actually going to draw it.
  if (!map_snapshots_enabled_ && (visualizer_ || anyUpdateMapSubscriber())) {
    map_snapshots_enabled_ = true;
    core_->setMapSnapshotPeriod(
      visualization_params_.map_update_decimation, visualization_params_.map_point_decimation);
  }

  double t_beg = 0, t_end = 0;
  bool usedPerPointTimes = false;
  auto cloud = toTrajLoCloud(*pc, fallback_scan_period_, t_beg, t_end, usedPerPointTimes);

  // Logged once: a continuous-time method run off synthesized azimuth times
  // is a large, silent accuracy loss, so which path is taken must be visible.
  if (!logged_time_source_) {
    logged_time_source_ = true;
    MRPT_LOG_INFO_STREAM(
      "Per-point timestamps: "
      << (usedPerPointTimes ? "taken from the cloud's own time field"
                            : "NOT present, synthesized from azimuth over fallback_scan_period=" +
                                std::to_string(fallback_scan_period_) + " s"));
  }

  // This only pushes into Traj-LO's own bounded queue; its processing thread
  // does the work.
  core_->processScan(cloud, t_beg, t_end);

  drainAndPublish();
}

void TrajLoOdometry::spinOnce() { drainAndPublish(); }

void TrajLoOdometry::drainAndPublish()
{
  if (!core_) {
    return;
  }
  for (const auto & out : core_->drainReady()) {
    publishOutput(out);
  }
}

void TrajLoOdometry::publishOutput(const traj_lo_core::OdometryOutput & out)
{
  // Traj-LO estimates the pose of the LiDAR frame itself (its map is built in
  // that frame and its `T_body_lidar` extrinsic is not used here), so this is
  // the `map -> lidar` convention the rest of the suite reports, unless a
  // base-link extrinsic was configured -- see lidar_pose_in_baselink_.
  mrpt::poses::CPose3D lidarPose = isometryToPose(out.pose);
  if (report_in_base_frame_) {
    lidarPose = lidarPose + lidar_pose_in_baselink_inv_;
  }
  const auto timestamp = mrpt::Clock::fromDouble(out.timestamp);

  {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    trajectory_.insert(timestamp, lidarPose);
  }

  if (anyUpdateLocalizationSubscriber()) {
    LocalizationUpdate lu;
    lu.timestamp = timestamp;
    lu.reference_frame = "map";
    lu.child_frame = report_in_base_frame_ ? "base_link" : "lidar";
    lu.method = "traj-lo";
    lu.pose = lidarPose.asTPose();
    advertiseUpdatedLocalization(lu);
  }

  if (anyUpdateMapSubscriber()) {
    const auto map = core_->currentMap();
    if (map && !map->empty()) {
      MapUpdate mu;
      mu.timestamp = timestamp;
      mu.reference_frame = "map";
      mu.method = "traj-lo";
      mu.map_name = "local_map";
      mu.map = toMrptPointsMap(*map);
      mu.keep_last_one_only = true;
      advertiseUpdatedMap(mu);
    }
  }

  updateVisualization(lidarPose, out);
}

void TrajLoOdometry::updateVisualization(
  const mrpt::poses::CPose3D & lidarPose, const traj_lo_core::OdometryOutput & out)
{
  if (!visualizer_) {
    return;
  }

  if (visualization_params_.current_pose_corner_size > 0) {
    auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
    glVehicle->insert(
      mrpt::opengl::stock_objects::CornerXYZ(visualization_params_.current_pose_corner_size));
    glVehicle->setPose(lidarPose);
    visualizer_->update_3d_object("trajlo/vehicle", glVehicle);
  }

  updateVisualizationPath(lidarPose);
  updateVisualizationMap();

  if (visualization_params_.camera_follows_vehicle) {
    const mrpt::math::TPoint3Df lookAt(lidarPose.x(), lidarPose.y(), lidarPose.z());
    visualizer_->update_viewport_look_at(lookAt);
  }

  if (!gui_created_) {
    internalBuildGUI();
    gui_created_ = true;
  }
  updateVisualizationTextLabels(out);
}

void TrajLoOdometry::updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose)
{
  if (!visualization_params_.show_trajectory) {
    visualizer_->update_3d_object("trajlo/path", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (!gl_estimated_path_) {
    gl_estimated_path_ = mrpt::opengl::CSetOfLines::Create();
    gl_estimated_path_->setColor_u8(0x20, 0xa0, 0x20, 0xff);
  }

  const auto t = lidarPose.translation();
  if (gl_estimated_path_->empty()) {
    gl_estimated_path_->appendLine(t, t);
  } else {
    gl_estimated_path_->appendLineStrip(t);
  }

  auto pathGrp = mrpt::opengl::CSetOfObjects::Create();
  pathGrp->insert(mrpt::opengl::CSetOfLines::Create(*gl_estimated_path_));
  visualizer_->update_3d_object("trajlo/path", pathGrp);
}

void TrajLoOdometry::updateVisualizationMap()
{
  if (!visualization_params_.show_map) {
    visualizer_->update_3d_object("trajlo/map", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  // The estimator refreshes this snapshot on its own thread, at
  // `map_update_decimation` emitted poses; there is nothing to decimate here.
  const auto mapPts = core_->currentMap();
  if (!mapPts || mapPts->empty()) {
    return;
  }
  if (mapPts == last_drawn_map_) {
    return;
  }
  last_drawn_map_ = mapPts;

  const auto pointsMap = toMrptPointsMap(*mapPts);

  auto glCloud = mrpt::opengl::CPointCloudColoured::Create();
  glCloud->loadFromPointsMap(pointsMap.get());
  glCloud->setPointSize(visualization_params_.map_point_size);

  const auto bbox = pointsMap->boundingBox();
  glCloud->recolorizeByCoordinate(bbox.min.z, bbox.max.z, 2 /*Z*/, mrpt::img::TColormap::cmJET);

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->insert(glCloud);
  visualizer_->update_3d_object("trajlo/map", grp);
}

void TrajLoOdometry::internalBuildGUI()
{
  using namespace mola::gui;

  gui_.lbStatus = std::make_shared<LiveString>(" ");

  Tab tab;
  tab.title = "Status";
  tab.widgets.emplace_back(Label{gui_.lbStatus});
  tab.widgets.emplace_back(CheckBox{
    "Show trajectory", visualization_params_.show_trajectory,
    [this](bool checked) { visualization_params_.show_trajectory = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Show local map", visualization_params_.show_map,
    [this](bool checked) { visualization_params_.show_map = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Camera follows vehicle", visualization_params_.camera_follows_vehicle,
    [this](bool checked) { visualization_params_.camera_follows_vehicle = checked; }});

  WindowDescription desc;
  desc.title = "Traj-LO";
  desc.position = {5, 700};
  desc.size = {340, 0};
  desc.tabs.emplace_back(std::move(tab));

  visualizer_->create_subwindow_from_description(desc);
}

void TrajLoOdometry::updateVisualizationTextLabels(const traj_lo_core::OdometryOutput & out)
{
  if (!gui_.lbStatus) {
    return;
  }

  const double speed = out.analytics.linear_velocity.norm();

  gui_.lbStatus->set(mrpt::format(
    "t=%.03f | seg err: %.4f inliers: %.0f iters: %d%s | thresh: %.2f m | voxels: %zu | "
    "speed: %.2f m/s | busy: %s",
    out.timestamp, out.analytics.last_error, out.analytics.last_inliers, out.analytics.iterations,
    out.analytics.converged ? "" : " (max)", out.analytics.reg_threshold, out.analytics.map_voxels,
    speed, isBusy() ? "yes" : "no"));
}

bool TrajLoOdometry::isBusy() const { return core_ && core_->isBusy(); }

mrpt::poses::CPose3DInterpolator TrajLoOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
