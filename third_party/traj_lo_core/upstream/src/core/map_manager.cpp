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

#include <trajlo/core/map_manager.h>

namespace traj {

std::vector<Eigen::Vector4d> MapManager::DownSampling(
    const std::vector<Eigen::Vector4d> &points, double ds_size) {
  tsl::robin_map<Voxel, Eigen::Vector4d, VoxelHash> grid;
  grid.reserve(points.size());

  for (const auto &point : points) {
    const auto voxel = Voxel((point.head<3>() / ds_size).cast<int>());
    if (grid.find(voxel) != grid.end()) continue;
    grid.insert({voxel, point});
  }
  std::vector<Eigen::Vector4d> ds_result;
  ds_result.reserve(grid.size());
  for (const auto &[voxel, point] : grid) {
    (void)voxel;
    ds_result.emplace_back(point);
  }
  return ds_result;
}
void MapManager::PreProcess(const std::vector<Eigen::Vector4d> &points,
                            const tStampPair &tp) {
  // pointcloud downsample
  map_points_database[tp] = DownSampling(points, ds_size_ * 0.5);
  reg_points_database[tp] =
      DownSampling(map_points_database[tp], ds_size_ * 1.5);
}

void MapManager::MapInit(const std::vector<Eigen::Vector4d> &points) {
  //    const auto& ds=downSampling(points, voxel_size );
  const auto &ds = points;

  std::for_each(ds.cbegin(), ds.cend(), [&](const auto &point) {
    Eigen::Vector3d p = point.head(3);
    auto voxel = Voxel((p / voxel_size_).template cast<int>());
    auto search = map.find(voxel);
    if (search != map.end()) {
      auto &voxel_block = search.value();
      voxel_block.AddPoint(p);
    } else {
      map.insert({voxel, VoxelBlock{{p}, max_points_per_voxel_}});
    }
  });
}

void MapManager::Update(const posePair &pp, const tStampPair &tp) {
  const auto &ds_points_map = map_points_database[tp];
  std::vector<Eigen::Vector3d> points_transformed(ds_points_map.size());
  Sophus::Vector6d tau = Sophus::se3_logd(pp.first.inverse() * pp.second);
  // tbb parallel for
  tbb::parallel_for(size_t(0), ds_points_map.size(), [&](size_t i) {
    Sophus::SE3d T_b_i = Sophus::se3_expd(ds_points_map[i].w() * tau);
    Sophus::SE3d T_w_i = pp.first * T_b_i;
    points_transformed[i] = T_w_i * ds_points_map[i].head<3>();
  });

  std::for_each(
      points_transformed.cbegin(), points_transformed.cend(),
      [&](const auto &point) {
        auto voxel = Voxel((point / voxel_size_).template cast<int>());
        auto search = map.find(voxel);
        if (search != map.end()) {
          auto &voxel_block = search.value();
          voxel_block.AddPoint(point);
        } else {
          map.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
      });

  // remove from current lidar origin
  //
  // MOLA port: upstream erased from the hash map while iterating it with a
  // range-for. tsl::robin_map's erase moves later elements to fill the hole,
  // which invalidates the running iterator, so voxels were skipped and the
  // traversal was undefined behavior. The keys are collected first and
  // erased afterwards; the set of voxels removed is the one upstream meant
  // to remove.
  const auto max_distance2 = max_range_ * max_range_;
  std::vector<Voxel> voxels_to_erase;
  for (const auto &kv : map) {
    // 使用voxel第一个remove
    const auto &pt = kv.second.points.front();
    if ((pt - pp.first.translation()).squaredNorm() > (max_distance2)) {
      voxels_to_erase.push_back(kv.first);
    }
  }
  for (const auto &voxel : voxels_to_erase) {
    map.erase(voxel);
  }

  // remove points out of temporal window
  map_points_database.erase(tp);
  reg_points_database.erase(tp);
}

void MapManager::PointRegistrationNormal(/*const posePair& pp,*/
                                         const posePairLin &ppl,
                                         const tStampPair &tp,
                                         Eigen::Matrix<double, 12, 12> &H_icp,
                                         Eigen::Matrix<double, 12, 1> &b_icp,
                                         double &error, double &inliers) {
  const double range_thresh = 3 * reg_thresh_;
  const double th = reg_thresh_ / 3;

  auto square = [](double x) { return x * x; };
  auto Weight = [&](double residual2) {
    return square(th) / (square(th) + residual2);
  };

  // real poses
  const posePair &pp{ppl.first.getPose(), ppl.second.getPose()};

  const Sophus::Vector6d tangent =
      Sophus::se3_logd(pp.first.inverse() * pp.second);

  /*
   * MOLA port: upstream evaluated "is either control pose already
   * linearized?" inside the parallel loop and, when it was, overwrote these
   * four enclosing-scope variables from every worker thread. That is a data
   * race, and it also made the result depend on how far each thread had got
   * before another one overwrote them, so the same input did not give the
   * same trajectory twice. The quantities do not depend on the point index,
   * so they are hoisted out of the loop here and evaluated once. The
   * residual and its correspondence search keep using the current (not
   * linearized) pose exactly as upstream does; only the Jacobian is taken at
   * the fixed linearization point, which is what the branch was for.
   */
  const bool at_lin_point =
      ppl.first.isLinearized() || ppl.second.isLinearized();

  const Sophus::SE3d T_w_b_jac =
      at_lin_point ? ppl.first.getPoseLin() : pp.first;
  const Sophus::Vector6d tangent_jac =
      at_lin_point ? Sophus::se3_logd(ppl.first.getPoseLin().inverse() *
                                      ppl.second.getPoseLin())
                   : tangent;
  const Eigen::Matrix3d R_w_b_t = T_w_b_jac.rotationMatrix().transpose();
  const Eigen::Matrix3d R_w_e_t =
      (at_lin_point ? ppl.second.getPoseLin() : pp.second)
          .rotationMatrix()
          .transpose();
  const Eigen::Matrix3d R_e_b = R_w_e_t * T_w_b_jac.rotationMatrix();

  const auto &ds_points_reg = reg_points_database[tp];

  /*
   * MOLA port: tbb::parallel_reduce splits and joins the per-range partial
   * sums in an order that depends on task stealing, so the accumulated
   * floating-point Hessian and gradient differ slightly between runs of the
   * same input, and the estimate diverges from there. The deterministic
   * variant fixes the split points and the reduction tree, making a run
   * bit-exact and independent of the thread count. Enabled by default; see
   * the TRAJLO_DETERMINISTIC option in the library's CMakeLists.txt.
   */
#ifdef TRAJLO_DETERMINISTIC
  const auto &[JTJ, JTr, e, num] = tbb::parallel_deterministic_reduce(
#else
  const auto &[JTJ, JTr, e, num] = tbb::parallel_reduce(
#endif
      // Range
      tbb::blocked_range<size_t>{0, ds_points_reg.size()},
      // Identity
      ResultTuple(),
      // 1st Lambda: Parallel computation
      [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
        for (auto i = r.begin(); i < r.end(); ++i) {
          double alpha = ds_points_reg[i].w();
          Sophus::SE3d T_b_i = Sophus::se3_expd(alpha * tangent);
          Sophus::SE3d T_w_i = pp.first * T_b_i;

          Eigen::Vector3d point = ds_points_reg[i].head<3>();
          Eigen::Vector3d p_in_world = T_w_i * point;

          auto kx = static_cast<int>(p_in_world[0] / voxel_size_);
          auto ky = static_cast<int>(p_in_world[1] / voxel_size_);
          auto kz = static_cast<int>(p_in_world[2] / voxel_size_);
          const auto key = Voxel(kx, ky, kz);

          std::vector<Voxel> voxels;
          voxels.reserve(7);
          for (const auto &c : coord) {
            voxels.emplace_back(key + c);
          }

          std::vector<Eigen::Vector3d> neighboors;
          neighboors.reserve(7 * max_points_per_voxel_);
          std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = map.find(voxel);
            if (search != map.end()) {
              const auto &points = search->second.points;
              if (!points.empty()) {
                for (const auto &point : points) {
                  neighboors.emplace_back(point);
                }
              }
            }
          });

          // find closet five point for normal estimation
          std::sort(neighboors.begin(), neighboors.end(),
                    [&](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
                      return (p1 - p_in_world).squaredNorm() <
                             (p2 - p_in_world).squaredNorm();
                    });
          if ((neighboors[0] - p_in_world).norm() > range_thresh) continue;
          if (neighboors.size() < 5) continue;

          Eigen::Matrix<double, 5, 3> A;
          Eigen::Matrix<double, 5, 1> b;
          b.setOnes();
          b *= -1.0f;
          for (int j = 0; j < 5; j++) {
            A.row(j) = neighboors[j];
          }
          Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);
          double n = normvec.norm();
          Eigen::Vector3d normal = normvec / n;
          bool flag = true;
          for (int j = 0; j < 5; j++) {
            if (fabs(normal.dot(neighboors[j]) + 1.0 / n) > planer_threshold_) {
              flag = false;
              break;
            }
          }
          if (!flag) continue;

          // point2plane use current state to compute the residual
          const Eigen::Vector3d residual = neighboors[0] - p_in_world;
          double w = Weight(residual.squaredNorm());
          Eigen::Matrix3d Information = normal * normal.transpose();

          if (normal.hasNaN() || residual.hasNaN()) {
            std::cout << "normal " << normal.transpose() << "   dis "
                      << residual.transpose() << std::endl;
            continue;
          }

          // if the pose relate to previous marginalized state, derive the Jacobian
          // at their fixed linearization point (hoisted, see above)
          if (at_lin_point) {
            T_b_i = Sophus::se3_expd(alpha * tangent_jac);
            T_w_i = T_w_b_jac * T_b_i;
          }

          Eigen::Matrix<double, 3, 6> J_T_wi;
          Eigen::Matrix<double, 6, 6> J_begin =
              Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 6> J_end =
              Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 12> J_be;

          J_T_wi.block<3, 3>(0, 0) = T_w_i.rotationMatrix();
          J_T_wi.block<3, 3>(0, 3) =
              -T_w_i.rotationMatrix() * Sophus::SO3d::hat(point);

          Eigen::Matrix3d Jr;
          Eigen::Matrix3d Jr_inv;

          Eigen::Vector3d omega = tangent_jac.tail<3>();
          Sophus::rightJacobianSO3(alpha * omega, Jr);
          Sophus::rightJacobianInvSO3(omega, Jr_inv);

          J_end.topLeftCorner<3, 3>() =
              (Sophus::SO3d::exp((1 - alpha) * omega)).matrix() * R_w_e_t;
          J_end.bottomRightCorner<3, 3>() = Jr * Jr_inv;

          // J-begin according to the chain rule to derive the Jacobian of the begin state
          Eigen::Matrix3d R_temp = Sophus::SO3d::exp(-alpha * omega).matrix();
          J_begin.topLeftCorner<3, 3>() = (1 - alpha) * R_temp * R_w_b_t;
          J_begin.bottomRightCorner<3, 3>() =
              R_temp - alpha * Jr * Jr_inv * R_e_b;

          J_be.block<6, 6>(0, 0) = J_begin;
          J_be.block<6, 6>(0, 6) = alpha * J_end;

          Eigen::Matrix<double, 3, 12> J_r;
          J_r = J_T_wi * J_be;  // 1*3 3*6 6*12;

          J.JTJ += w * J_r.transpose() * Information * J_r;
          J.JTr += w * J_r.transpose() * Information * residual;
          J.inlier += 1;
          J.error += abs(normal.dot(residual));
        }
        return J;
      },
      // 2nd Lambda: Parallel reduction of the private Jacboians
      [&](ResultTuple a, const ResultTuple &b) -> ResultTuple {
        return a + b;
      });

  H_icp = JTJ;
  b_icp = JTr;
  error = e;
  inliers = num;
}

/*
 * MOLA port addition: a decimated copy of the map's points, for the host
 * framework's map visualization. Iteration order of the underlying hash map
 * is arbitrary but stable, so the decimation keeps a consistent subset
 * between calls.
 */
void MapManager::Snapshot(std::vector<Eigen::Vector3d> &out,
                          int decimation) const {
  if (decimation < 1) decimation = 1;
  out.clear();
  out.reserve(map.size());
  size_t i = 0;
  for (const auto &kv : map) {
    for (const auto &p : kv.second.points) {
      if (i++ % static_cast<size_t>(decimation) == 0) out.emplace_back(p);
    }
  }
}

}  // namespace traj
