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
 * @file   TrajLoOdometry_Adapters.cpp
 * @brief  Conversions between MRPT/MOLA and traj_lo_core plain types
 */
#include "TrajLoOdometry_Adapters.h"

#include <mrpt/maps/CSimplePointsMap.h>

#include <algorithm>
#include <cmath>

namespace mola
{
using traj_lo_core::LidarPoint;
using traj_lo_core::LidarPointVector;
using traj_lo_core::XYZIPointVector;

/** Converts an incoming LiDAR observation into Traj-LO points, and computes the
 * scan's absolute [t_beg, t_end] time span. Same strategy as
 * `mola_fast_lio2_wrapper`'s `toFastLio2Cloud()` and
 * `mola_dlio_wrapper`'s `toDlioCloud()`, so the three wrappers see the same
 * per-point timing on the same input: any per-point timestamp channel is
 * re-normalized against this scan's own min/max, and when there is none, a
 * synthetic offset is derived from each point's azimuth.
 *
 * `traj_lo_core::LidarPoint::t_offset_sec` is always relative to `t_beg`, so
 * this is the single place in this wrapper responsible for the timestamp
 * convention. Unlike the other two, no near-field cut is applied here:
 * Traj-LO has its own `min_range` filter, applied per segment rather than per
 * scan, and using it keeps the parameter's meaning identical to upstream's.
 *
 * `usedPerPointTimes` reports which of the two paths ran, so the module can log
 * it once: taking the azimuth fallback on a sensor that does carry real
 * per-point times is a silent, large accuracy loss for a continuous-time
 * method.
 */
LidarPointVector toTrajLoCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, bool & usedPerPointTimes)
{
  LidarPointVector out;
  usedPerPointTimes = false;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    t_beg = t_end = mrpt::Clock::toDouble(obs.timestamp);
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();

  const auto * is =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  out.resize(n);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  if (ts && ts->size() == n && n > 0) {
    usedPerPointTimes = true;

    float tMin = (*ts)[0], tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }
    t_beg = obsTime + tMin;
    t_end = obsTime + tMax;

    for (size_t i = 0; i < n; i++) {
      LidarPoint & p = out[i];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i] - tMin);
    }
  } else {
    // Fallback: assume `obsTime` is the scan's end (forward-facing point),
    // and synthesize each point's offset from its azimuth angle.
    t_beg = obsTime - fallback_scan_period;
    t_end = obsTime;

    for (size_t i = 0; i < n; i++) {
      LidarPoint & p = out[i];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;

      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t_offset_sec = azimuth01 * fallback_scan_period;
    }
  }

  return out;
}

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const XYZIPointVector & points)
{
  auto out = mrpt::maps::CSimplePointsMap::Create();
  out->reserve(points.size());
  for (const auto & p : points) {
    out->insertPointFast(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace mola
