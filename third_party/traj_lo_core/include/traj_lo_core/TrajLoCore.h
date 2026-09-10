// SPDX-License-Identifier: GPL-3.0-only
// mola_traj_lo_wrapper: original code (not derived from Traj-LO). See
// THIRD_PARTY_NOTICES.md at the repository root.
#pragma once

#include <cstddef>
#include <memory>
#include <traj_lo_core/Types.hpp>
#include <vector>

namespace traj_lo_core
{
/** Thin, ROS-free and MOLA-free owner of `traj::TrajLOdometry`.
 *
 * Only plain data crosses this boundary (see `Types.hpp`): no Sophus, no TBB,
 * no `tsl::robin_map`, so a consumer needs none of the vendored include dirs.
 *
 * Threading: Traj-LO's own processing thread and its bounded input queue are
 * part of the method's design and are kept. This class does not add a thread
 * of its own; `processScan()` pushes into that queue and `isBusy()` reports
 * whether anything pushed so far is still unconsumed, which is what the
 * offline CLI's no-drop loop waits on.
 */
class TrajLoCore
{
public:
  explicit TrajLoCore(const Config & cfg);
  ~TrajLoCore();

  TrajLoCore(const TrajLoCore &) = delete;
  TrajLoCore & operator=(const TrajLoCore &) = delete;

  /** Feeds one LiDAR sweep. `t_beg` / `t_end` are absolute times [s] on the
   * caller's clock; each point's `t_offset_sec` is relative to `t_beg`.
   * Returns false (and drops the scan) if it carries no points.
   *
   * Poses are not produced synchronously: a segment's control pose is emitted
   * only once it leaves the sliding window, so expect the first output after
   * roughly `init_interval + seg_num * seg_interval` of data.
   */
  bool processScan(const LidarPointVector & points, double t_beg, double t_end);

  /** True while any scan fed so far is still queued or being processed. */
  bool isBusy() const;

  /** Removes and returns the control poses emitted since the last call. */
  std::vector<OdometryOutput> drainReady();

  /** Flushes the estimator: stops its processing thread, which first emits the
   * control poses still inside the sliding window. Call before the last
   * `drainReady()` to get the tail of the trajectory. Idempotent.
   */
  void finish();

  /** Most recent decimated snapshot of the local map, in world frame, or
   * nullptr if none was taken yet. Refreshed on the processing thread, so no
   * lock is held by the caller; see `setMapSnapshotPeriod()`.
   */
  std::shared_ptr<const XYZIPointVector> currentMap() const;

  /** How many emitted control poses to wait between local-map snapshots.
   * 0 (the default) disables snapshotting entirely, which is what the offline
   * CLI wants: nothing draws the map there and copying it is not free.
   */
  void setMapSnapshotPeriod(int everyNPoses, int pointDecimation = 4);

  /** Absolute time [s] that the estimator's internal zero corresponds to, set
   * from the first scan fed. Traj-LO works in `int64_t` nanoseconds; rebasing
   * onto the first scan keeps sub-microsecond resolution regardless of whether
   * the dataset's clock starts at 0 (KITTI) or at a Unix epoch (rosbags).
   * Zero until the first scan has been fed.
   */
  double sessionEpoch() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace traj_lo_core
