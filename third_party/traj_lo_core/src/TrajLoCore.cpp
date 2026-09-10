// SPDX-License-Identifier: GPL-3.0-only
// mola_traj_lo_wrapper: original code (not derived from Traj-LO). See
// THIRD_PARTY_NOTICES.md at the repository root.

#include <traj_lo_core/TrajLoCore.h>
#include <trajlo/core/odometry.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace traj_lo_core
{
namespace
{
/// One-line-per-segment dump of what geometry alone is doing, enabled by
/// setting MOLA_TRAJLO_DUMP_SEGMENT_DIAGNOSTICS. Off by default and zero cost
/// when off. Traj-LO has no IMU and therefore no gravity estimate, so this
/// takes the place of the other wrappers' MOLA_*_DUMP_ESTIMATED_GRAV trace.
/// The variable's value is the output file name, or just a truthy flag, in
/// which case the name below is used.
std::string dumpFileName()
{
  const char * v = ::getenv("MOLA_TRAJLO_DUMP_SEGMENT_DIAGNOSTICS");
  if (v == nullptr || v[0] == '\0') {
    return {};
  }
  const std::string s = v;
  if (s == "1" || s == "true" || s == "yes" || s == "on") {
    return "trajlo_segment_diagnostics.txt";
  }
  return s;
}

/// Roll and pitch of a rotation matrix in the yaw-pitch-roll (Z-Y-X)
/// convention MRPT and the rest of this benchmark suite use.
void rollPitchFrom(const Eigen::Matrix3d & R, double & roll, double & pitch)
{
  pitch = std::atan2(-R(2, 0), std::hypot(R(0, 0), R(1, 0)));
  roll = std::atan2(R(2, 1), R(2, 2));
}

Eigen::Isometry3d toIsometry(const Sophus::SE3d & T)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = T.rotationMatrix();
  out.translation() = T.translation();
  return out;
}

}  // namespace

struct TrajLoCore::Impl
{
  explicit Impl(const Config & cfg) : cfg_(cfg)
  {
    traj::TrajConfig tc;
    tc.init_interval = cfg.init_interval;
    tc.seg_interval = cfg.seg_interval;
    tc.seg_num = cfg.seg_num;
    tc.kinematic_constrain = cfg.kinematic_constrain;
    tc.init_pose_weight = cfg.init_pose_weight;
    tc.converge_thresh = cfg.converge_thresh;
    tc.max_iterations = cfg.max_iterations;
    tc.ds_size = cfg.ds_size;
    tc.voxel_size = cfg.voxel_size;
    tc.max_voxel_num = cfg.max_voxel_num;
    tc.planer_thresh = cfg.planer_thresh;
    tc.max_range = cfg.max_range;
    tc.min_range = cfg.min_range;

    odom_ = std::make_unique<traj::TrajLOdometry>(tc);
    odom_->pose_callback = [this](
                             int64_t t_ns, const Sophus::SE3d & T_wc,
                             const traj::SegmentDiagnostics & diag) { onPose(t_ns, T_wc, diag); };
    odom_->Start();
  }

  ~Impl() { finish(); }

  void finish()
  {
    if (finished_.exchange(true)) {
      return;
    }
    // The null scan is the estimator's own stop sentinel: its processing
    // thread breaks out of the loop, emits the control poses still inside
    // the sliding window, and sets isFinish.
    odom_->laser_data_queue.push(nullptr);
    while (!odom_->isFinish.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void onPose(int64_t t_ns, const Sophus::SE3d & T_wc, const traj::SegmentDiagnostics & diag)
  {
    OdometryOutput out;
    out.timestamp = epoch_ + static_cast<double>(t_ns) * 1e-9;
    out.pose = toIsometry(T_wc);
    out.analytics.last_error = diag.lastError;
    out.analytics.last_inliers = diag.lastInliers;
    out.analytics.iterations = diag.iterations;
    out.analytics.converged = diag.converged;
    out.analytics.map_voxels = diag.map_voxels;
    out.analytics.window_size = diag.window_size;
    out.analytics.reg_threshold = diag.reg_threshold;
    out.analytics.linear_velocity = diag.twist.head<3>();
    out.analytics.angular_velocity = diag.twist.tail<3>();

    if (dump_file_) {
      double roll = 0, pitch = 0;
      rollPitchFrom(out.pose.linear(), roll, pitch);
      (*dump_file_) << out.timestamp << " " << diag.lastError << " " << diag.lastInliers << " "
                    << diag.iterations << " " << diag.twist(0) << " " << diag.twist(1) << " "
                    << diag.twist(2) << " " << diag.twist(3) << " " << diag.twist(4) << " "
                    << diag.twist(5) << " " << pitch << " " << roll << "\n";
    }

    if (map_snapshot_period_ > 0 && ++poses_since_snapshot_ >= map_snapshot_period_) {
      poses_since_snapshot_ = 0;
      takeMapSnapshot();
    }

    {
      std::lock_guard<std::mutex> lck(ready_mtx_);
      ready_.emplace_back(std::move(out));
    }
  }

  void takeMapSnapshot()
  {
    std::vector<Eigen::Vector3d> pts;
    odom_->MapSnapshot(pts, map_point_decimation_);

    auto snapshot = std::make_shared<XYZIPointVector>();
    snapshot->reserve(pts.size());
    for (const auto & p : pts) {
      XYZIPoint q;
      q.x = static_cast<float>(p.x());
      q.y = static_cast<float>(p.y());
      q.z = static_cast<float>(p.z());
      snapshot->emplace_back(q);
    }

    std::lock_guard<std::mutex> lck(map_mtx_);
    map_ = std::move(snapshot);
  }

  Config cfg_;
  std::unique_ptr<traj::TrajLOdometry> odom_;

  double epoch_ = 0;
  bool epoch_set_ = false;
  std::atomic_bool finished_{false};

  mutable std::mutex ready_mtx_;
  std::vector<OdometryOutput> ready_;

  mutable std::mutex map_mtx_;
  std::shared_ptr<const XYZIPointVector> map_;
  std::atomic_int map_snapshot_period_{0};
  std::atomic_int map_point_decimation_{4};
  int poses_since_snapshot_ = 0;

  // Opened lazily on construction so that the "off" case never touches disk.
  std::unique_ptr<std::ofstream> dump_file_ = [] {
    const std::string fil = dumpFileName();
    if (fil.empty()) {
      return std::unique_ptr<std::ofstream>();
    }
    auto f = std::make_unique<std::ofstream>(fil);
    (*f) << "# t_sec lastError lastInliers iterations vx vy vz wx wy wz pitch_rad roll_rad\n";
    return f;
  }();
};

TrajLoCore::TrajLoCore(const Config & cfg) : impl_(std::make_unique<Impl>(cfg)) {}
TrajLoCore::~TrajLoCore() = default;

bool TrajLoCore::processScan(const LidarPointVector & points, double t_beg, double t_end)
{
  (void)t_end;
  if (points.empty()) {
    return false;
  }

  if (!impl_->epoch_set_) {
    // Rebased onto a whole second so that the offset is exactly
    // representable and the same scan always lands on the same nanosecond
    // tick regardless of how long the recording is.
    impl_->epoch_ = std::floor(t_beg);
    impl_->epoch_set_ = true;
  }

  const double base = t_beg - impl_->epoch_ + impl_->cfg_.time_offset;

  auto scan = std::make_shared<traj::Scan>();
  scan->points.resize(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    auto & q = scan->points[i];
    const auto & p = points[i];
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.intensity;
    q.ts = base + p.t_offset_sec;
  }

  // Traj-LO cuts a sweep into segments by walking its points once and
  // closing a segment when a point's time crosses the boundary, so the
  // points must arrive in ascending time order. Sensor drivers deliver them
  // that way, but a dataset reader that reorders points -- or the azimuth
  // fallback used when a cloud carries no time field at all -- does not, and
  // an out-of-order point would be silently dropped into the wrong segment.
  if (!std::is_sorted(
        scan->points.begin(), scan->points.end(),
        [](const traj::PointXYZIT & a, const traj::PointXYZIT & b) { return a.ts < b.ts; })) {
    std::stable_sort(
      scan->points.begin(), scan->points.end(),
      [](const traj::PointXYZIT & a, const traj::PointXYZIT & b) { return a.ts < b.ts; });
  }

  scan->size = scan->points.size();
  scan->timestamp = static_cast<int64_t>(scan->points.front().ts * 1e9);

  impl_->odom_->pending_scans++;
  impl_->odom_->laser_data_queue.push(scan);
  return true;
}

bool TrajLoCore::isBusy() const { return impl_->odom_->pending_scans.load() > 0; }

std::vector<OdometryOutput> TrajLoCore::drainReady()
{
  std::vector<OdometryOutput> out;
  std::lock_guard<std::mutex> lck(impl_->ready_mtx_);
  out.swap(impl_->ready_);
  return out;
}

void TrajLoCore::finish() { impl_->finish(); }

std::shared_ptr<const XYZIPointVector> TrajLoCore::currentMap() const
{
  std::lock_guard<std::mutex> lck(impl_->map_mtx_);
  return impl_->map_;
}

void TrajLoCore::setMapSnapshotPeriod(int everyNPoses, int pointDecimation)
{
  impl_->map_snapshot_period_ = everyNPoses;
  impl_->map_point_decimation_ = std::max(1, pointDecimation);
}

double TrajLoCore::sessionEpoch() const { return impl_->epoch_; }

}  // namespace traj_lo_core
