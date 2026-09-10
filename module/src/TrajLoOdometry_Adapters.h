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
 * @file   TrajLoOdometry_Adapters.h
 * @brief  Conversions between MRPT/MOLA and traj_lo_core plain types (private
 *         to this module, not installed).
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <traj_lo_core/Types.hpp>

namespace mola
{
/** Where a converted scan's per-point times came from. Reported out of
 * toTrajLoCloud() so the module can log it once: Traj-LO is a continuous-time
 * method, so per-point timing is the input it segments on, and silently taking
 * the wrong path costs accuracy without failing.
 */
enum class ScanTimeSource
{
  /// The cloud's own time field, holding offsets relative to the scan.
  PerPointRelative,
  /// The cloud's own time field, holding absolute times on the sensor clock.
  PerPointAbsolute,
  /// No time field at all: offsets synthesized from each point's azimuth.
  AzimuthFallback,
  /// There was a time field, but it carried no usable intra-sweep information:
  /// every point of the scan had collapsed onto one value, or the span it
  /// covered was too small for the field's own resolution. Offsets synthesized
  /// from azimuth, as above.
  AzimuthAfterUnusableField,
};

/** See the .cpp file for the full documentation of the conversion logic. */
traj_lo_core::LidarPointVector toTrajLoCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, ScanTimeSource & timeSource);

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const traj_lo_core::XYZIPointVector & points);

}  // namespace mola
