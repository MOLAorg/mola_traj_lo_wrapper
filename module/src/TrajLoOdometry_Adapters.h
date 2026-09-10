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
/** See the .cpp file for the full documentation of the conversion logic. */
traj_lo_core::LidarPointVector toTrajLoCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, bool & usedPerPointTimes);

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const traj_lo_core::XYZIPointVector & points);

}  // namespace mola
