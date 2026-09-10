// SPDX-License-Identifier: GPL-3.0-only
// mola_traj_lo_wrapper: standalone test of traj_lo_core, with no MOLA, MRPT or
// ROS dependency.

#include <gtest/gtest.h>
#include <traj_lo_core/TrajLoCore.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace
{
constexpr double kScanPeriod = 0.1;   // 10 Hz
constexpr double kSpeed = 0.5;        // [m/s] along +X
constexpr double kT0 = 1700000000.0;  // a Unix-epoch-sized start time
constexpr int kNumScans = 40;

// A closed rectangular room. Traj-LO is LiDAR-only, so the scene has to
// constrain all six degrees of freedom by geometry alone: a room gives three
// pairs of non-parallel planes, which does.
constexpr double kRoomX = 20.0, kRoomY = 20.0, kRoomZ = 6.0;
constexpr double kSampleStep = 0.25;

/// World-frame sensor position at absolute time `t`: constant velocity along
/// +X, starting at the middle of the room.
Eigen::Vector3d sensorAt(double t)
{
  return {kRoomX * 0.5 + kSpeed * (t - kT0), kRoomY * 0.5, kRoomZ * 0.5};
}

std::vector<Eigen::Vector3d> roomSurface()
{
  std::vector<Eigen::Vector3d> pts;
  for (double a = 0; a <= kRoomX; a += kSampleStep) {
    for (double b = 0; b <= kRoomY; b += kSampleStep) {
      pts.emplace_back(a, b, 0.0);
      pts.emplace_back(a, b, kRoomZ);
    }
  }
  for (double a = 0; a <= kRoomX; a += kSampleStep) {
    for (double c = 0; c <= kRoomZ; c += kSampleStep) {
      pts.emplace_back(a, 0.0, c);
      pts.emplace_back(a, kRoomY, c);
    }
  }
  for (double b = 0; b <= kRoomY; b += kSampleStep) {
    for (double c = 0; c <= kRoomZ; c += kSampleStep) {
      pts.emplace_back(0.0, b, c);
      pts.emplace_back(kRoomX, b, c);
    }
  }
  return pts;
}

/// One sweep, in the sensor frame, genuinely distorted by the motion: each
/// point is expressed in the sensor pose of its own timestamp.
traj_lo_core::LidarPointVector makeScan(
  const std::vector<Eigen::Vector3d> & surface, double tBeg, double minRange)
{
  traj_lo_core::LidarPointVector out;
  out.reserve(surface.size());

  const double n = static_cast<double>(surface.size());
  for (size_t i = 0; i < surface.size(); i++) {
    const double dt = kScanPeriod * static_cast<double>(i) / n;
    const Eigen::Vector3d local = surface[i] - sensorAt(tBeg + dt);
    if (local.norm() < minRange) {
      continue;
    }
    traj_lo_core::LidarPoint p;
    p.x = static_cast<float>(local.x());
    p.y = static_cast<float>(local.y());
    p.z = static_cast<float>(local.z());
    p.intensity = 1.0f;
    p.t_offset_sec = dt;
    out.emplace_back(p);
  }
  return out;
}

traj_lo_core::Config makeConfig()
{
  traj_lo_core::Config cfg;
  cfg.max_range = 60.0f;
  cfg.min_range = 0.5f;
  return cfg;
}

/// Feeds the whole synthetic sequence through one estimator instance, waiting
/// on isBusy() between scans exactly as the offline CLI does.
std::vector<traj_lo_core::OdometryOutput> runSequence()
{
  const auto surface = roomSurface();
  const auto cfg = makeConfig();
  traj_lo_core::TrajLoCore core(cfg);

  std::vector<traj_lo_core::OdometryOutput> all;
  for (int i = 0; i < kNumScans; i++) {
    const double tBeg = kT0 + kScanPeriod * i;
    const auto scan = makeScan(surface, tBeg, cfg.min_range);
    EXPECT_FALSE(scan.empty());
    core.processScan(scan, tBeg, tBeg + kScanPeriod);

    while (core.isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    for (auto & o : core.drainReady()) {
      all.emplace_back(std::move(o));
    }
  }

  core.finish();
  for (auto & o : core.drainReady()) {
    all.emplace_back(std::move(o));
  }
  return all;
}

}  // namespace

TEST(TrajLoCore, TracksConstantVelocityMotion)
{
  const auto poses = runSequence();

  // Traj-LO emits one control pose per segment boundary, not one per scan, so
  // there must be clearly more of them than there were scans.
  ASSERT_GT(poses.size(), static_cast<size_t>(kNumScans));

  for (size_t i = 1; i < poses.size(); i++) {
    EXPECT_GE(poses[i].timestamp, poses[i - 1].timestamp);
  }

  // Timestamps must come back on the input clock, not on the estimator's own
  // zero-based nanosecond one.
  EXPECT_NEAR(poses.front().timestamp, kT0, 1.0);
  EXPECT_GT(poses.back().timestamp, kT0 + 3.0);

  // The estimator's world frame is its first control pose, so the expected
  // pose is the motion accumulated since then.
  const double tRef = poses.front().timestamp;
  const auto & last = poses.back();
  const double expectedX = kSpeed * (last.timestamp - tRef);

  EXPECT_NEAR(last.pose.translation().x(), expectedX, 0.30);
  EXPECT_NEAR(last.pose.translation().y(), 0.0, 0.30);
  EXPECT_NEAR(last.pose.translation().z(), 0.0, 0.30);

  // Pure translation: the orientation must stay near identity.
  const double angle = Eigen::AngleAxisd(last.pose.linear()).angle();
  EXPECT_LT(std::abs(angle), 0.15);

  // The registration actually found correspondences.
  size_t withInliers = 0;
  for (const auto & p : poses) {
    if (p.analytics.last_inliers > 0) {
      withInliers++;
    }
  }
  EXPECT_GT(withInliers, poses.size() / 2);
}

TEST(TrajLoCore, IsReproducible)
{
  const auto a = runSequence();
  const auto b = runSequence();

  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++) {
    EXPECT_EQ(a[i].timestamp, b[i].timestamp) << " at " << i;
    EXPECT_TRUE(a[i].pose.matrix() == b[i].pose.matrix()) << " at " << i;
    EXPECT_EQ(a[i].analytics.last_inliers, b[i].analytics.last_inliers) << " at " << i;
    EXPECT_EQ(a[i].analytics.last_error, b[i].analytics.last_error) << " at " << i;
  }
}
