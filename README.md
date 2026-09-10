# mola_traj_lo_wrapper

MOLA wrapper for **Traj-LO**, a continuous-time, **LiDAR-only** odometry method.

> X. Zheng and J. Zhu, *Traj-LO: In Defense of LiDAR-Only Odometry Using an
> Effective Continuous-Time Trajectory*, IEEE RA-L, 2024.
> Upstream: <https://github.com/kevin2431/Traj-LO>

It provides two entry points over the same estimator:

- `mola::TrajLoOdometry`, a MOLA front-end module loadable from a `mola-cli`
  launch YAML, with live 3-D visualization; and
- `mola-traj-lo-cli`, an offline, loss-free, **deterministic** CLI for
  benchmarking against other SLAM/odometry methods on identical input.

Both read their input through MOLA dataset sources, so the same build runs on
KITTI, Oxford Spires, GrandTour, MulRan, ROS 1 / ROS 2 bags and MRPT rawlogs.

This package is GPL-3.0. It bundles Traj-LO, Sophus and robin-map, all MIT ---
see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the exact commits, what
was dropped, and every change made to the vendored algorithm.

---

## What is different about this method

Traj-LO is the LiDAR-only arm of this benchmark suite. Three things follow from
that and from its continuous-time formulation, and all three affect how its
output should be read:

- **No IMU, anywhere.** There is no `--imu-topic` flag, the launch files declare
  no IMU sensor, and there is no gravity or bias estimate to dump. That is the
  method, not an omission: it measures what pure geometry achieves with no
  inertial anchoring at all.
- **No pose per scan.** Each sweep is cut into segments of `seg_interval`
  (40 ms by default, against a ~100 ms sweep), and the control poses at the
  segment boundaries are estimated jointly over a sliding window of `seg_num`
  of them. Poses therefore come out at ~25 Hz rather than at the 10 Hz scan
  rate, and a given scan may yield several or none. The TUM file has more rows
  than the dataset has scans.
- **A pose is reported only once it leaves the window**, after it has been
  refined and then marginalized. In the GUI the drawn trajectory therefore lags
  the newest scan by about `seg_num * seg_interval`, and its tail visibly
  adjusts as the window slides. That is correct behavior.

Upstream's own README warns that the method "may fail in narrow spaces when
there are few valid points to get geometric constraints". A divergence on a
given sequence is a legitimate result for this method, not necessarily a bug in
this port.

---

## Build

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select mola_traj_lo_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Dependencies are Eigen 3, TBB, MRPT and the MOLA kernel; Sophus and robin-map
are vendored, and there is no PCL and no yaml-cpp dependency. The dataset input
packages (`mola_input_rosbag2`, `mola_input_rosbag1`,
`mola_input_kitti_dataset`, `mola_input_mulran_dataset`, `mola_input_rawlog`)
are each optional: CMake reports which ones were found as
`mola-traj-lo-cli: With <X> = YES/NO`.

### Determinism

The offline CLI is reproducible: two runs over the same dataset with the same
pipeline YAML produce byte-identical output. Three things make that true, and
all three are needed:

1. `traj_lo_core` is built with `TRAJLO_DETERMINISTIC` (a CMake option, **on by
   default**), which replaces `tbb::parallel_reduce` with
   `tbb::parallel_deterministic_reduce` in the point registration. Upstream's
   reduction order depends on task stealing, so its floating-point Hessian and
   gradient differ slightly between runs and the estimate diverges from there.
   The result is also independent of the thread count.
2. A data race in the same loop was fixed --- see change 6 in
   [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
3. The CLI feeds one scan at a time and waits on `isBusy()` before the next, so
   nothing is dropped and the estimator sees the same sequence every time.

`test_traj_lo_core` asserts the reproducibility directly. Configure with
`-DTRAJLO_DETERMINISTIC=OFF` to restore upstream's behavior exactly; that test
then fails, which is the point of it.

The **online** module (`mola-cli`) makes no such guarantee: there, scans arrive
at whatever rate the dataset source or the live sensor delivers them, and a
dropped or late scan changes the result.

---

## Running: offline CLI

```
mola-traj-lo-cli -c <pipeline.yaml> <one dataset input flag> [--output-tum-path out.tum]
```

Dataset input flags (whichever were compiled in): `--input-rosbag2`,
`--input-rosbag1`, `--input-kitti-seq`, `--input-mulran-seq`, `--input-rawlog`.
Both bag flags accept a comma-separated list of bags, replayed back to back as
one continuous sequence, and take `--lidar-topic`.

Other flags: `--only-first-n N` (stop after N dataset *entries* --- note that a
bag entry is one message of any topic, not one scan), `-v DEBUG`,
`--load-plugins`.

When the dataset provides a ground-truth trajectory, it is written next to the
output as `<name>_gt.<ext>`.

### Oxford Spires

```bash
SEQ=/mnt/datasets/public/oxford-spires/2024-03-13-observatory-quarter-01
BAG=$SEQ/raw/ros2bag/1710338090_2024-03-13-13-54-51

mola-traj-lo-cli \
  -c $(ros2 pkg prefix mola_traj_lo_wrapper)/share/mola_traj_lo_wrapper/pipelines/trajlo-oxford-spires.yaml \
  --input-rosbag2 "$BAG" \
  --lidar-topic /hesai/pandar \
  --output-tum-path /tmp/trajlo_oq01.tum

evo_ape tum $SEQ/processed/trajectory/gt-tum.txt /tmp/trajlo_oq01.tum \
  --align --t_max_diff 0.05
```

No `LIDAR_POSE_*` / `MOLA_USE_FIXED_LIDAR_POSE` environment variables are needed
here, unlike for the LiDAR-inertial wrappers: nothing in this method consumes a
sensor extrinsic. The frame the trajectory is *reported* in is set by the
pipeline's `baselink2lidar_pose_str` instead --- see below.

### KITTI

```bash
export KITTI_BASE_DIR=/path/to/kitti   # holding sequences/00/...
mola-traj-lo-cli -c .../pipelines/trajlo-kitti.yaml \
  --input-kitti-seq 00 --output-tum-path /tmp/trajlo_kitti00.tum
```

KITTI needs no wrapping here, unlike in `mola-dlio-cli`, which has to interleave
a synthetic IMU stream because DLIO cannot run without one. Two caveats specific
to KITTI are documented at the top of `pipelines/trajlo-kitti.yaml`: its clouds
carry no per-point time field (so the azimuth fallback runs) and they are
already provider-deskewed (so the continuous-time deskewing has little left to
do).

### GrandTour

```bash
M=/mnt/datasets/public/grand-tour/2024-12-09-09-34-43
N=$(basename $M)

MOLA_TF_BASE_LINK=base \
mola-traj-lo-cli -c .../pipelines/trajlo-grandtour.yaml \
  --input-rosbag1 "$M/${N}_hesai_undist.bag,$M/${N}_tf_minimal.bag" \
  --lidar-topic /boxi/hesai/points_undistorted \
  --output-tum-path /tmp/trajlo_grandtour.tum
```

`MOLA_TF_BASE_LINK=base` is **required**: this rig's base frame is called
`base`, not `base_link`, and without it every TF lookup fails and every scan is
silently dropped (the run then reports `LiDAR scans fed: 0`). The launch file
sets it already; only the CLI needs it passed. The extrinsics themselves come
from the mission's own `*_tf_minimal.bag`, which is why it is joined to the
LiDAR bag. The mission's `*_adis.bag` is not needed at all.

---

## Running: online, in `mola-cli`

```bash
# Oxford Spires, with the multipart-sequence helper:
mola-trajlo-gui-oxford-spires /mnt/datasets/public/oxford-spires/2024-03-13-observatory-quarter-01/
mola-trajlo-gui-oxford-spires <seq> --headless      # no GUI

# Or directly:
export MOLA_INPUT_ROSBAG1=<mission>_hesai_undist.bag
mola-cli $(ros2 pkg prefix mola_traj_lo_wrapper)/share/mola_traj_lo_wrapper/mola-cli-launchs/trajlo_from_grandtour.yaml
```

Launch files: `trajlo_from_oxford_spires.yaml` (rosbag2),
`trajlo_from_grandtour.yaml` (rosbag1), `trajlo_from_kitti.yaml`. Common
overrides: `MOLA_WITH_GUI=false`, `MOLA_TIME_WARP`,
`MOLA_QUIT_ON_DATASET_END=true`, `MOLA_MAX_DURATION_SEC`.

The GUI draws the current pose, the estimated trajectory, the local voxel map
and a status sub-window (segment residual, inlier count, iteration count,
adaptive threshold, voxel count, current speed).

To export a trajectory from the online path, set `save_trajectory_to_file: true`
and `trajectory_output_file:` in the pipeline YAML (or export
`MOLA_SAVE_TRAJECTORY=true` and `MOLA_TUM_TRAJECTORY_OUTPUT=...`).

---

## Parameters

Pipeline YAMLs live in `pipelines/`. The algorithm parameters keep upstream's
own names and meanings (upstream's `trajectory:` and `mapping:` blocks), so
upstream's documentation applies; the tunable ones are exposed as
`${ENV_VAR|default}`.

| Parameter | Default | What it does |
|---|---|---|
| `init_interval` | `3e8` ns | Length of the very first segment: long enough to initialize the map from a full sweep. |
| `seg_interval` | `4e7` ns | Length of every subsequent segment. The sub-scan rate of the whole method. |
| `seg_num` | `3` | Control poses in the sliding window. |
| `kinematic_constrain` | `2.0` | Weight of the constant-twist constraint between consecutive segments, scaled by the segment's inlier count. |
| `init_pose_weight` | `1e9` | Prior pinning the first control pose. |
| `converge_thresh` | `0.001` | Gauss-Newton stop threshold on the max increment. |
| `max_iterations` | `25` | Gauss-Newton iteration cap per segment. |
| `ds_size` | `0.4` m | Input downsampling; map insertion uses `ds_size*0.5`, registration `ds_size*1.5`. |
| `voxel_size` | `0.4` m | Local map voxel size. |
| `max_voxel_num` | `20` | Points kept per map voxel. |
| `planer_thresh` | `0.1` | Planarity gate on the 5-NN plane fit; smaller is stricter. |
| `max_range` | `100` m | Map radius; also scales the adaptive registration threshold. |
| `min_range` | `1.0` m | Near-field cut. Upstream's default is 0.3; on handheld/backpack and legged rigs the carrier itself is a false zero-motion constraint. |
| `time_offset` | `0.0` s | Added to every per-point time before segmentation. |

Wrapper-level parameters:

| Parameter | Default | What it does |
|---|---|---|
| `lidar_sensor_label` | `lidar` | Regex matched against the observation's sensor label. |
| `baselink2lidar_pose_str` | `0 0 0 0 0 0` | `T_base_lidar`, as `x y z yaw_deg pitch_deg roll_deg`. When not identity, the reported trajectory is right-composed with its inverse, so it is `map -> base_link` instead of `map -> lidar`. |
| `fallback_scan_period` | `0.1` s | Only used when a cloud has no per-point time field. |
| `save_trajectory_to_file`, `trajectory_output_file` | off | TUM export from the online path. |
| `show_trajectory`, `show_map`, `camera_follows_vehicle`, `current_pose_corner_size`, `map_point_size`, `map_update_decimation`, `map_point_decimation` | | Visualization. `map_update_decimation` is counted in emitted control poses. |

### Frames, and why `baselink2lidar_pose_str` exists

Traj-LO estimates the pose of the LiDAR frame itself. Some datasets give their
ground truth in a different base frame: Oxford Spires' `gt-tum.txt` is in a
`base` frame that is the raw LiDAR frame yawed 180 degrees and lifted 0.124 m.
An APE computed with `--align` barely notices such a body-frame offset, but it
negates the horizontal part of every relative motion, so relative drift and tilt
metrics would be measuring the frame convention instead of the odometry. Setting
`baselink2lidar_pose_str` makes the module report in the base frame directly,
rather than leaving it to a post-processing step. `pipelines/trajlo-oxford-spires.yaml`
sets it to `0 0 0.124 180 0 0`; the other pipelines leave it at identity.

### Per-point timestamps

Traj-LO is a continuous-time method, so per-point timing *is* the input it cuts
each sweep into segments on. It matters more here than in the LiDAR-inertial
wrappers, where a bad per-point time costs some deskewing accuracy but the IMU
still carries the motion.

The adapter handles three cases, and says which one ran, once, on the first
scan:

| Log line | What it means |
|---|---|
| `the cloud's own time field (scan-relative offsets)` | The normal path. The field's own convention (starting at zero, mid-scan-centered, ...) does not matter: it is re-normalized against that scan's own min/max. |
| `the cloud's own time field (absolute times)` | The field holds absolute times on the sensor clock; the scan's span is taken from the field rather than from the observation's stamp. |
| `none in the cloud, synthesizing them from azimuth` (WARN) | No time field. Each point's offset comes from its azimuth over `fallback_scan_period`. Expected on KITTI; on a sensor that does publish per-point times, the topic or reader configuration is wrong. |
| `HAS a time field, but it carries no usable intra-sweep timing` (WARN) | See below. |

That last case is the one worth knowing about. A time field can be present and
still be empty of information, and handing a continuous-time estimator a scan of
zero duration makes every segment of that sweep degenerate --- it then drifts
without anything failing. Two ways it happens, both seen on real data:

- MRPT holds per-point times as `float`. A field holding **absolute Unix times**
  near 1.7e9 has a float spacing of about 128 s, so a whole sweep quantizes onto
  one value.
- A dataset reader that cannot decode the field's datatype may leave every point
  at zero. `mrpt::ros1bridge`'s PointCloud2 conversion did exactly this for
  `FLOAT64` time fields, which is what GrandTour's Hesai publishes --- fixed in
  [MRPT/mrpt_ros_bridge#9](https://github.com/MRPT/mrpt_ros_bridge/pull/9).
  Without that fix, this wrapper falls back to azimuth and warns rather than
  running on zero-duration sweeps, but the real per-point times are better.

Both are caught by requiring the scan's span to cover at least a few
representable steps of the field itself.

---

## Instrumentation

Traj-LO has no IMU and therefore no gravity initialization to trace, so it has
no `MOLA_*_DUMP_ESTIMATED_GRAV` equivalent. What it dumps instead is what
geometry alone is doing per segment:

```bash
export MOLA_TRAJLO_DUMP_SEGMENT_DIAGNOSTICS=trajlo_segments.txt   # or =1 for a default name
```

One line per emitted control pose:

```
# t_sec lastError lastInliers iterations vx vy vz wx wy wz pitch_rad roll_rad
```

`(vx..wz)` is the twist implied by the segment's two control poses, in the
begin-pose frame --- Traj-LO carries no explicit velocity state, and this is its
closest analogue. `pitch`/`roll` are the control pose's own attitude, which is
the quantity to compare against the IMU-anchored methods in the suite: for a
LiDAR-only method, how the geometry alone constrains roll and pitch over time is
the interesting trace. Off by default, zero cost when off.

The same quantities are available programmatically in
`traj_lo_core::OdometryOutput::analytics` (see
`third_party/traj_lo_core/include/traj_lo_core/Types.hpp`), which also carries
the map's voxel count, the sliding window's size and the adaptive registration
threshold.

---

## Layout

```
apps/mola-traj-lo-cli.cpp              offline CLI
module/                                mola::TrajLoOdometry + MRPT adapters
pipelines/                             algorithm parameters, per dataset
mola-cli-launchs/                      online launch files, per dataset
scripts/mola-trajlo-gui-oxford-spires  multipart-sequence launcher
third_party/traj_lo_core/
    include/traj_lo_core/              the plain-data API boundary
    src/TrajLoCore.cpp                 owner of traj::TrajLOdometry
    test/                              standalone gtest, no MOLA/MRPT/ROS
    upstream/                          vendored Traj-LO
third_party/{Sophus,robin-map}/        vendored header-only dependencies
```

`traj_lo_core` depends on nothing from MOLA, MRPT or ROS, and only plain data
crosses its API. Sophus, robin-map and TBB are private to it, so consumers of
this package need none of them.
