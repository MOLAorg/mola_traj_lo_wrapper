/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TRAJLO_ODOMETRY_H
#define TRAJLO_ODOMETRY_H

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <tbb/concurrent_queue.h>
#include <Eigen/Eigen>

#include <trajlo/utils/common_type.h>
#include <trajlo/utils/config.h>
#include <trajlo/utils/pose_type.h>
#include <trajlo/utils/eigen_utils.hpp>

#include <trajlo/core/map_manager.h>

namespace traj {

/*
 * MOLA port addition. Diagnostics of the segment whose control pose is being
 * emitted, gathered at the same point where the pose leaves the sliding
 * window. Everything here is already computed by the estimator; nothing is
 * recomputed for it.
 */
struct SegmentDiagnostics {
  double lastError = 0;    // sum of point-to-plane residuals of the segment
  double lastInliers = 0;  // number of point-to-plane correspondences
  int iterations = 0;      // Gauss-Newton iterations actually run
  bool converged = false;  // whether converge_thresh was reached before max_iterations
  size_t map_voxels = 0;   // occupied voxels in the local map
  size_t window_size = 0;  // segments currently in the sliding window
  double reg_threshold = 0;         // adaptive point-registration threshold
  Sophus::Vector6d twist =          // (v, omega) implied by the segment's two
      Sophus::Vector6d::Zero();     // control poses, in the begin-pose frame [m/s, rad/s]
};

class TrajLOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TrajLOdometry>;

  /*
   * MOLA port addition. Invoked, from the processing thread, at each of the
   * three places where upstream appends to `trajectory_`: the standing-start
   * initialization, every marginalization, and the final flush of the poses
   * still inside the window when the stream ends. `trajectory_` itself is
   * kept unchanged.
   */
  using PoseCallback = std::function<void(
      int64_t t_ns, const Sophus::SE3d& T_wc, const SegmentDiagnostics& diag)>;
  PoseCallback pose_callback;

  /*
   * MOLA port addition. Number of scans pushed into `laser_data_queue` but
   * not yet fully consumed. The producer increments it before each push; the
   * processing thread decrements it after the scan has been segmented,
   * optimized and marginalized. A queue-size check alone cannot express
   * that, since a scan is off the queue while it is being processed.
   */
  std::atomic<int> pending_scans{0};

  TrajLOdometry(const TrajConfig& config);
  ~TrajLOdometry() {
    // MOLA port: Start() may never have been called; upstream dereferenced
    // the thread pointer unconditionally.
    if (processing_thread_ && processing_thread_->joinable()) {
      processing_thread_->join();
    }
  }

  void Start();
  void Optimize();
  void Marginalize();
  void PointCloudSegment(Scan::Ptr scan, Measurement::Ptr measure);
  void RangeFilter(Measurement::Ptr measure, std::vector<Eigen::Vector4d>& points);

  /* MOLA port addition: decimated copy of the local map, in world frame,
   * taken on the processing thread so no lock is needed on `map_`. */
  void MapSnapshot(std::vector<Eigen::Vector3d>& out, int decimation) const;

  tbb::concurrent_bounded_queue<Scan::Ptr> laser_data_queue;
  std::atomic<bool> isFinish;  // for feed data thread join

 private:
  MapManager::Ptr map_;
  bool isMove_ = false;

  Sophus::SE3d T_wc_curr;
  Sophus::SE3d T_prior;  // motion prior
  Eigen::aligned_map<tStampPair, Measurement::Ptr> measurements;
  std::deque<Measurement::Ptr> measure_cache;
  std::vector<PointXYZIT> points_cache;

  double min_range_2_=1.0;
  double converge_thresh_=0.01;

  // pose database
  Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses_;
  using tumPose = std::pair<int64_t, Sophus::SE3d>;
  std::vector<tumPose> trajectory_;
  //  std::vector<Sophus::SE3d> trajectory;

  // slide window parameters
  int64_t window_interval_ = 4e7;
  int64_t init_interval_ = 3e8;
  size_t max_frames_ = 4;
  int64_t last_begin_t_ns_;
  int64_t last_end_t_ns_;


  // MOLA port addition: filled by Optimize(), reported through pose_callback.
  int last_iterations_ = 0;
  bool last_converged_ = false;

  // marginalization
  AbsOrderMap marg_order;
  Eigen::MatrixXd marg_H;
  Eigen::VectorXd marg_b;

  TrajConfig config_;

  // backend thread
//  bool initialized_;
  std::shared_ptr<std::thread> processing_thread_;
};

}  // namespace traj

#endif  // TRAJLO_ODOMETRY_H
