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
#include <limits>

namespace mola
{
using traj_lo_core::LidarPoint;
using traj_lo_core::LidarPointVector;
using traj_lo_core::XYZIPointVector;

namespace
{
/// A scan-relative per-point offset spans at most one sweep, so a time field
/// whose smallest value exceeds this is holding absolute times on some epoch
/// instead. [s]
constexpr double kAbsoluteTimeThreshold = 1e3;

/// How many representable float steps a scan's time span must cover for the
/// field to carry usable intra-sweep timing.
constexpr float kMinTimeQuantaPerScan = 16.0f;
}  // namespace

/** Converts an incoming LiDAR observation into Traj-LO points, and computes the
 * scan's absolute [t_beg, t_end] time span. `traj_lo_core::LidarPoint::t_offset_sec`
 * is always relative to `t_beg`, so this is the single place in this wrapper
 * responsible for the timestamp convention.
 *
 * Per-point timing matters more here than in this suite's other wrappers:
 * Traj-LO is a continuous-time method, and these offsets are what it cuts each
 * sweep into segments on. Three cases, in order:
 *
 * 1. **A time field holding scan-relative offsets** (Oxford Spires' Hesai, and
 *    most drivers). Whatever the field's own convention -- starting at zero,
 *    centered on the scan's middle -- it is re-normalized against this scan's
 *    own min/max, and the span is anchored on the observation's timestamp. Same
 *    as `mola_fast_lio2_wrapper`'s `toFastLio2Cloud()` and `mola_dlio_wrapper`'s
 *    `toDlioCloud()`, so the three wrappers see identical timing on identical
 *    input.
 * 2. **A time field holding absolute times** (GrandTour's Hesai publishes
 *    `timestamp` as a float64 Unix time). The span comes from the field itself
 *    rather than from the observation's stamp, since the two are the same clock
 *    and the field is the more precise of the two.
 * 3. **No usable time field.** Each point's offset is synthesized from its
 *    azimuth over `fallback_scan_period`, treating the observation's timestamp
 *    as the scan's end.
 *
 * A time field that is *present but useless* falls into case 3 as well, and
 * detecting that matters more here than it would for a scan-to-map method:
 * handing a continuous-time estimator a scan of zero duration makes every
 * segment of that sweep degenerate, and it drifts without ever failing. Two
 * ways it happens, both seen on real data: MRPT holds per-point times as
 * `float`, and a Unix-epoch value near 1.7e9 has a float32 spacing of ~128 s,
 * so a whole sweep quantizes onto one value; and a dataset reader that cannot
 * decode the field's datatype may leave every point at zero. Both are caught by
 * requiring the scan's span to cover at least a few representable steps of the
 * field itself.
 *
 * No near-field cut is applied here, unlike in the other two wrappers:
 * Traj-LO has its own `min_range` filter, applied per segment rather than per
 * scan, and using it keeps that parameter's meaning identical to upstream's.
 */
LidarPointVector toTrajLoCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, ScanTimeSource & timeSource)
{
  LidarPointVector out;
  timeSource = ScanTimeSource::AzimuthFallback;

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

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  bool usableTimes = ts && ts->size() == n && n > 0;
  float tMin = 0, tMax = 0;
  if (usableTimes) {
    tMin = (*ts)[0];
    tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }

    // How far apart two representable values are at this magnitude. A field
    // holding a Unix-epoch time is spaced ~128 s here, a thousand times a whole
    // sweep, so its intra-sweep timing is simply not in there any more. A
    // reader that fails to decode the field at all leaves every point at the
    // same value, which fails the same check.
    const float quantum = std::nextafterf(tMax, std::numeric_limits<float>::infinity()) - tMax;
    const bool spanIsUsable = tMax - tMin > quantum * kMinTimeQuantaPerScan;

    if (!spanIsUsable) {
      timeSource = ScanTimeSource::AzimuthAfterUnusableField;
      usableTimes = false;
    } else if (static_cast<double>(tMin) > kAbsoluteTimeThreshold) {
      timeSource = ScanTimeSource::PerPointAbsolute;
      t_beg = tMin;
      t_end = tMax;
    } else {
      timeSource = ScanTimeSource::PerPointRelative;
      t_beg = obsTime + tMin;
      t_end = obsTime + tMax;
    }
  }

  out.resize(n);

  if (usableTimes) {
    for (size_t i = 0; i < n; i++) {
      LidarPoint & p = out[i];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i]) - static_cast<double>(tMin);
    }
  } else {
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
