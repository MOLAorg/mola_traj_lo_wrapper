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

#ifndef TRAJLO_CONFIG_H
#define TRAJLO_CONFIG_H

/*
 * MOLA port: the dataset/topic/path, pose-saving, ground-truth-frame and
 * visualization fields were removed together with the yaml-cpp loader
 * (src/utils/config.cpp) -- the host framework owns I/O and configuration,
 * and populates this struct field by field. The defaults below are the ones
 * upstream ships in data/config_hesai.yaml, so a default-constructed
 * TrajConfig reproduces upstream's Hesai settings.
 */

namespace traj {
struct TrajConfig {
  TrajConfig() = default;

  // trajectory
  double init_interval = 3e8;   // [ns]
  double seg_interval = 4e7;    // [ns]
  int seg_num = 3;
  float kinematic_constrain = 2.0f;
  double init_pose_weight = 1e9;
  double converge_thresh = 0.001;
  int max_iterations = 25;

  // mapping
  float ds_size = 0.4f;
  float voxel_size = 0.4f;
  int max_voxel_num = 20;
  float planer_thresh = 0.1f;
  float max_range = 120.0f;
  float min_range = 0.3f;
};
}  // namespace traj

#endif  // TRAJLO_CONFIG_H
