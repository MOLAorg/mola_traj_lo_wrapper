/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   test_scan_conversion.cpp
 * @brief  Per-point timing conventions of the MRPT -> traj_lo_core adapter.
 *
 * These are pure data conversions with no estimator involved, and they are
 * where this port is easiest to get silently wrong: Traj-LO is a
 * continuous-time method, so a wrong per-point time convention does not fail,
 * it just degrades the trajectory.
 */
#include <gtest/gtest.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <cmath>

#include "TrajLoOdometry_Adapters.h"

namespace
{
constexpr double kObsTime = 1710338090.25;
constexpr double kScanPeriod = 0.1;

/// A cloud whose per-point time field is centered on the scan's middle, one of
/// the conventions found in the wild (the adapter must not assume it starts at
/// zero).
mrpt::obs::CObservationPointCloud::Ptr makeCloudWithTimes()
{
  auto pts = mrpt::maps::CGenericPointsMap::Create();
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->timestamp = mrpt::Clock::fromDouble(kObsTime);

  pts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  pts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
  pts->resize(10);
  for (size_t i = 0; i < 10; i++) {
    pts->setPointFast(i, 1.0f + static_cast<float>(i), 2.0f, 3.0f);
    pts->setPointField_float(i, mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY, 0.5f);
    pts->setPointField_float(
      i, mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP,
      static_cast<float>(-kScanPeriod * 0.5 + kScanPeriod * static_cast<double>(i) / 10.0));
  }
  obs->pointcloud = pts;
  return obs;
}

mrpt::obs::CObservationPointCloud::Ptr makeCloudWithoutTimes()
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->timestamp = mrpt::Clock::fromDouble(kObsTime);

  // Four points, one per quadrant, so their azimuths are known.
  pts->insertPointFast(-1.0f, -0.001f, 0.0f);  // azimuth ~ -pi  -> offset ~ 0
  pts->insertPointFast(0.0f, -1.0f, 0.0f);     // azimuth = -pi/2 -> offset = T/4
  pts->insertPointFast(1.0f, 0.0f, 0.0f);      // azimuth = 0     -> offset = T/2
  pts->insertPointFast(0.0f, 1.0f, 0.0f);      // azimuth = +pi/2 -> offset = 3T/4
  obs->pointcloud = pts;
  return obs;
}

}  // namespace

TEST(ScanConversion, PerPointTimesAreRebasedOntoScanStart)
{
  const auto obs = makeCloudWithTimes();

  double t_beg = 0, t_end = 0;
  bool usedPerPointTimes = false;
  const auto out = mola::toTrajLoCloud(*obs, kScanPeriod, t_beg, t_end, usedPerPointTimes);

  EXPECT_TRUE(usedPerPointTimes);
  ASSERT_EQ(out.size(), 10u);

  // The span is the observation time plus the field's own min/max, so a
  // mid-scan-centered field puts t_beg *before* the observation stamp.
  EXPECT_NEAR(t_beg, kObsTime - kScanPeriod * 0.5, 1e-6);
  EXPECT_NEAR(t_end, kObsTime + kScanPeriod * 0.4, 1e-6);

  // Offsets must be relative to t_beg, hence non-negative and starting at 0.
  EXPECT_NEAR(out.front().t_offset_sec, 0.0, 1e-6);
  for (size_t i = 0; i < out.size(); i++) {
    EXPECT_GE(out[i].t_offset_sec, 0.0);
    EXPECT_NEAR(out[i].t_offset_sec, kScanPeriod * i / 10.0, 1e-6);
  }

  // XYZI must survive the conversion untouched.
  EXPECT_NEAR(out[3].x, 4.0f, 1e-5);
  EXPECT_NEAR(out[3].y, 2.0f, 1e-5);
  EXPECT_NEAR(out[3].z, 3.0f, 1e-5);
  EXPECT_NEAR(out[3].intensity, 0.5f, 1e-5);
}

TEST(ScanConversion, FallsBackToAzimuthWhenThereIsNoTimeField)
{
  const auto obs = makeCloudWithoutTimes();

  double t_beg = 0, t_end = 0;
  bool usedPerPointTimes = true;
  const auto out = mola::toTrajLoCloud(*obs, kScanPeriod, t_beg, t_end, usedPerPointTimes);

  EXPECT_FALSE(usedPerPointTimes);
  ASSERT_EQ(out.size(), 4u);

  // With no time field the observation stamp is taken as the scan's end.
  EXPECT_NEAR(t_beg, kObsTime - kScanPeriod, 1e-6);
  EXPECT_NEAR(t_end, kObsTime, 1e-6);

  EXPECT_NEAR(out[0].t_offset_sec, 0.0, 2e-3);
  EXPECT_NEAR(out[1].t_offset_sec, kScanPeriod * 0.25, 1e-6);
  EXPECT_NEAR(out[2].t_offset_sec, kScanPeriod * 0.50, 1e-6);
  EXPECT_NEAR(out[3].t_offset_sec, kScanPeriod * 0.75, 1e-6);

  for (const auto & p : out) {
    EXPECT_GE(p.t_offset_sec, 0.0);
    EXPECT_LE(p.t_offset_sec, kScanPeriod);
  }
}

TEST(ScanConversion, EmptyCloudDoesNotCrash)
{
  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->timestamp = mrpt::Clock::fromDouble(kObsTime);

  double t_beg = 0, t_end = 0;
  bool usedPerPointTimes = true;
  const auto out = mola::toTrajLoCloud(*obs, kScanPeriod, t_beg, t_end, usedPerPointTimes);

  EXPECT_TRUE(out.empty());
  EXPECT_NEAR(t_beg, kObsTime, 1e-6);
  EXPECT_NEAR(t_end, kObsTime, 1e-6);
}
