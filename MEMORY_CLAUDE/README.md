# RTAB-Map integration — work log

Record of how RTAB-Map was brought into the WiL stack, why each choice was made,
and the numbers measured along the way. Written to support a later write-up: the
*root causes* and the *measurements* are the parts worth quoting, so they are
kept verbatim rather than summarised.

Fork HEAD at time of writing: `594ad97 unfinish rtab build`, branch `wil`,
built on upstream `2eef2b3 Lyrical support (#1433)`.

Core library: **apt `ros-humble-rtabmap` 0.23.7**, not built from source.
Wrappers: **this fork**, built by colcon in the workspace.


---

## 0. Fixes, in one table

Every defect found and what closed it. Details in the numbered sections; the
chronology there also records the wrong turns, deliberately.

| # | Symptom | Root cause | Fix | Where |
|---|---|---|---|---|
| 1 | Build failed: "could not find RTABMap 0.23.10" | Fork's `ros2` tip needs 0.23.10; apt ships 0.23.7 | Pin fork to `2eef2b3` (newest ament commit needing only 0.23.5). **`38b45dc^` is a ROS 1 catkin tree** -- the `ros2` branch merges `master`, so `^` walks off the ROS 2 line | §3 |
| 2 | `rtabmap_odom` failed on missing deps | Building all 14 packages | `colcon build --packages-up-to rtabmap_slam` -- 5 packages. `rtabmap_odom` is the internal VO we are REPLACING | §4 |
| 3 | CMake error on `pcl_ros` | Only genuinely required system dep | `apt install ros-humble-pcl-ros`. octomap/grid_map/apriltag/aruco are optional `find_package()` | §4 |
| 4 | tf2 logged **1724** "jump back in time"; nothing resolved | Bags RECORD `/clock` (74180 msgs); `ros2 bag play --clock` publishes a second, wall-epoch clock. Disagreement ~1.79e9 s | **Play these bags WITHOUT `--clock`** | §6 |
| 5 | "Did not receive data since 5 seconds"; 3 nodes per 45 s run | Odometry TOPIC at 40 Hz in a 5-topic ApproximateTime set with ~3 Hz images; the match is evicted | Take pose from **TF** (`odom_frame_id`), leaving only the 4 image/camera_info topics in the sync. Also `topic_queue_size` 10 -> 200 | §6 |
| 6 | Blocked waiting on `ackermann_robot_001/...` TF | RTAB-Map's `imu` input is a RELATIVE topic name, silently resolving to the bag's `/imu` | Remap `imu` -> `/rtabmap/imu_unused`; the front-end already fuses IMU | §6 |
| 7 | Occupancy grid tilted out of horizontal, not aligned to the path | Graph had **zero gravity links**: `Optimizer/GravitySigma` 0.3 can only preserve an orientation the graph carries, and `Mem/UseOdomGravity` is false upstream | `Mem/UseOdomGravity: true` -- VINS is stereo-inertial and gravity-aligned. 0 -> 164 gravity links | §6 |
| 8 | `vio.csv` had one timestamp repeated on every row | `/mapPath` has two publishers: the live one (`CoreWrapper.cpp:4879`) stamps every pose with the same message time | Call `/rtabmap/publish_map` on a timer; only accept Paths whose stamps differ (`CoreWrapper.cpp:4266`) | §6 |
| 9 | `-d` erased a results directory | `database_path` accepted a directory, and the node runs with `-d` | Resolve any non-`.db` value to `<path>/rtabmap.db`; added `delete_db` arg | §6 |
| 10 | Nothing visible in RViz | `rtabmap_viz`/`rtabmap_rviz_plugins` not built, and the launch started no visualiser | `rviz:=true` + `rtab_map.rviz` using stock displays. `/map`, `/cloud_map`, `/mapPath` are plain ROS types -- no plugins needed | §4 |
| 11 | Map covered only one keyframe | RTAB-Map takes pose from TF; no front-end running means no `world -> body` | Run the front-end first and wait for it to initialise. `tf2_echo world body` is the check | §6c |
| 12 | Frames dropped; TF lagged 11-15 s | VINS `proc_ms` ~114 vs 100 ms frame spacing at `--rate 0.5`: a growing backlog | Play at `--rate 0.3`; raised `wait_for_transform` 0.3 -> 5.0 s | §6c |
| 14 | `grid_range_max` cropped half the scene | 4.0 m was chosen to suppress smears that were fix 13; measured, far points are mostly real | Default 4.0 -> **10.0** (recall 45 % -> 51 %, precision 94 % -> 89 %) | §6c-M6 |
| 13 | **Grid was smears, not walls; 9 loop closures** | **`cv2.FileStorage` silently misread `body_T_cam*`**: declared `dt: d` but whole numbers written bare (`0`, `1`, `-1`), so only decimal-valued entries survived. Translation read perfectly, **rotation came back ~1e9-1e18**. Published TF normalised to a camera pitched ~34 deg | Parse the `data: [...]` list from text; validate every matrix with `_check_rigid()` (orthonormal, det +1, else exit). **72 loop closures, precision 42 % -> 91 %, chamfer p95 1.85 -> 0.35 m** | §6, §6c-M5 |

**The one to remember is #13.** It failed invisibly -- the shim started cleanly and
printed the correct 93.6 mm baseline, because that comes from the translations that
parsed fine. Nothing downstream checks that a transform is rigid, and every
conclusion drawn about map quality before it was found had to be retracted.
Validate transforms read from files.

---

## 1. Why RTAB-Map at all

The VINS fork carries no loop fusion, so its trajectories drift monotonically
with nothing to correct them. RTAB-Map is added as a **back-end behind** the
existing front-ends, not as a replacement for either:

```
                ┌─→ VINS odom ──────┐
stereo + IMU ───┤                   ├──→ rtabmap ──→ loop closure + graph opt
                └─→ ORB-SLAM3 odom ─┘
```

It contributes appearance-based loop detection (DBoW over incoming images),
pose-graph optimisation when a loop fires, and an assembled map. Holding the
back-end fixed while swapping only the odometry source isolates what each
front-end contributes — a cleaner claim than comparing two systems whose
back-ends also differ.

**The old CARLA attempt is not reusable.** `vins_fusion_ros2/src/carla_rtab_direct.cpp`
links VINS and RTAB-Map into one binary, which is why `vins_fusion_ros2/CMakeLists.txt`
(lines 54–59) marks it deprecated and gates it behind `WITH_RTABMAP=OFF`: apt
rtabmap is built against a different OpenCV than `vins_lib`. That conflict is
**link-time only**. Running them as separate ROS nodes puts them in separate
address spaces, so the apt core is fine and the CPU-twin-OpenCV `rtab_tool/`
workaround (which no longer exists on this machine) is unnecessary.

---

## 2. This fork is the wrappers only

`introlab/rtabmap_ros` and `introlab/rtabmap` are **two separate repositories**.
This fork is the first: 14 ROS 2 packages, no `corelib/`, `guilib/` or `utilite/`.
`rtabmap_conversions/CMakeLists.txt` calls `find_package(RTABMap ...)` for the
second, which must come from somewhere else.

Two coherent options; the third is what was chosen:

| | core | wrappers |
|---|---|---|
| all-apt | `ros-humble-rtabmap` | `ros-humble-rtabmap-ros` (fork unused) |
| all-source | clone `introlab/rtabmap` | this fork |
| **chosen** | **`ros-humble-rtabmap` 0.23.7** | **this fork** |

Do **not** also install `ros-humble-rtabmap-ros` — it is the apt build of these
same wrappers and collides with the source build.

---

## 3. Version pinning, and the ROS1/ROS2 branch trap

The fork's `ros2` tip (`1e6edb4`) requires **RTABMap 0.23.10**; apt ships
**0.23.7**. Build fails at `rtabmap_conversions/CMakeLists.txt:33`.

Relaxing the version number does not work. `38b45dc "Update for rtabmap 0.23.10 (#1442)"`
is only 26 lines but uses API absent from 0.23.7:

- `rtabmap::PythonInterface::instance()` — new static factory replacing the old
  stack object
- `Parameters::kg2oPixelVariance()` → renamed `kOptimizerPixelVariance()`

**The trap: `38b45dc^` is a ROS 1 tree.** The `ros2` branch periodically merges
ROS 1 `master` into itself, so first-parent traversal walks off the ROS 2 line.
`b51ba87` (the parent) has `<buildtool_depend>catkin</buildtool_depend>`, ships
`rtabmap_legacy/`, and references `nodelet` — colcon cannot build it at all, and
the failure mode is far more confusing than the version error it "fixes". A
checkout to `b51ba87` was made and then corrected.

Verify a candidate on **both** axes before pinning:

```bash
git show <c>:rtabmap_conversions/package.xml   | grep build_type      # want ament_cmake
git show <c>:rtabmap_conversions/CMakeLists.txt | grep find_package\(RTABMap  # want <= 0.23.7
```

Scanning first-parent history of `origin/ros2` gives the boundaries:

| commit | date | build type | requires |
|---|---|---|---|
| `1e6edb4` | 2026-08-26 | ament | 0.23.10 |
| **`2eef2b3`** | **2026-06-21** | **ament** | **0.23.5** ← chosen |
| `fb27ac0` | 2026-04-18 | ament | 0.23.4 |
| `d63accf` | 2025-12-20 | ament | 0.22.0 |

`2eef2b3` is the newest ROS 2 commit still satisfied by apt 0.23.7. Everything
after it on `ros2` is the 0.23.10 merge and CI changes.

**Housekeeping:** `594ad97` on this branch accidentally committed colcon
artifacts (`log/COLCON_IGNORE`, `log/latest`, two `logger_all.log`, ~1700 lines).
Worth reverting before the fork is pushed anywhere.

---

## 4. Build only five packages

```bash
colcon build --packages-up-to rtabmap_slam
```

`rtabmap_msgs`, `rtabmap_conversions`, `rtabmap_util`, `rtabmap_sync`,
`rtabmap_slam` — not all 14.

**`rtabmap_odom` is deliberately excluded.** It is RTAB-Map's own visual
odometry, i.e. exactly the component being replaced by VINS/ORB-SLAM3. The
build failure that started this work (`Failed to find package.sh ... rtabmap_sync,
rtabmap_util`) was `rtabmap_odom` failing on dependencies it never needed to have.
`rtabmap_viz` (Qt) is also skipped, and so is `rtabmap_rviz_plugins`. Neither is
needed to watch the map build: the node's `/map`, `/cloud_map` and `/mapPath` are
plain `OccupancyGrid` / `PointCloud2` / `Path`, which `rviz_default_plugins`
renders directly. `rtab_map.rviz` is a stock-display config for exactly that, and
`rtab_sim.launch.py rviz:=true` opens it. Note RTAB-Map only *assembles* clouds
and grids when something subscribes, so nothing is computed until RViz asks — and
the map refreshes at `Rtabmap/DetectionRate` (1 Hz), which looks far chunkier
than a full-rate VO map.

`rtabmap_launch` compiles nothing but its `exec_depend`s pull in `rtabmap_odom`,
`rtabmap_viz` and `rtabmap_rviz_plugins`, so adding it defeats the point.

**Only one system dependency is genuinely required:**

```bash
sudo apt install ros-humble-pcl-ros      # 2.4.5
```

`pcl_ros` is `find_package(... REQUIRED)` at `rtabmap_util/CMakeLists.txt:25`.
`octomap_msgs`, `grid_map_ros`, `apriltag_msgs`, `aruco_msgs` and
`aruco_opencv_msgs` are plain `find_package()` guarded by `IF(..._FOUND)` — they
emit CMake *warnings* and disable features (octomap export, grid_map output,
fiducial landmarks) that this project does not use. `rtabmap_util` finishing in
2 min 15 s with those two warnings is a **success**, not a failure.

An earlier `rtabmap_sync` failure logged `gmake: *** Interrupt` — that is Ctrl+C,
not a compile error. `rtabmap_sync` is the slow package (heavy template
instantiation in `CommonDataSubscriberRGBD2…6.cpp`); let it finish.

---

## 5. What the sim bags do not carry

`dataset/static_dataset_loop` holds only `/clock`, `/imu`, `/cmd_vel`,
`/ground_truth/odometry`, `/cam0/image_raw/compressed`, `/cam1/image_raw/compressed`.
RTAB-Map's stereo path additionally needs `camera_info` for both cameras, TF from
the body frame to the camera optical frames, and decompressed images. All three
are manufactured by **`rtab_camera_shim.py`** from the same files VINS reads, so
there is one source of truth:

```
vins_fusion_ros2/config/wil_sim/cam0_pinhole.yaml   intrinsics
vins_fusion_ros2/config/wil_sim/cam1_pinhole.yaml
vins_fusion_ros2/config/wil_sim/stereo_imu.yaml     body_T_cam0 / body_T_cam1
```

Derived and verified:

| quantity | value |
|---|---|
| resolution | 1280 × 720 |
| fx = fy | 640.0 |
| cx, cy | 640.0, 360.0 |
| distortion | all terms 0.0 |
| cam0 in body | (0.158, **+0.0936**, 0.2475) |
| cam1 in body | (0.158, **0.0**, 0.2475) |
| baseline | **93.6 mm** |
| right `P[0][3]` = −fx·B | **−59.904** |
| body→cam optical quaternion | (0.5, −0.5, 0.5, −0.5) |

`script/check_stereo_side.py dataset/static_dataset_loop` returns
`median disparity = +11.94 px, 9/9 positive → cam1 is the RIGHT camera`,
independently confirming what the extrinsics imply (body is FLU, so cam0 at
y = +0.0936 is further left). **cam0 = left, cam1 = right.**

The images are published as `image_rect` **without any rectification step**. A gz
camera is an ideal pinhole — every distortion term is 0.0 and both cameras share
intrinsics — so the raw frames already are what rectification would produce. The
shim asserts this and exits if any distortion term is non-zero, because the same
shortcut on the real rig would silently corrupt every triangulated point.

Both frames are stamped with the **left** frame's time (they are written ~15 ms
apart in the bag). This matches what `orbslam3_node.cpp` already does, so the two
systems stay comparable.

---

## 6. Bugs found, and their root causes

Ordered as encountered. All were silent or misleading.

**`--clock` on a bag that already records `/clock`.** These bags contain 74,180
recorded `/clock` messages. `ros2 bag play --clock` publishes a *second* clock
from the player, on wall-epoch time, while the recorded one carries sim time
starting at ~69 s. The two disagree by ~1.79 × 10⁹ s. tf2 logged **1724**
"Detected jump back in time. Clearing TF buffer", and RTAB-Map could never
resolve a camera transform. **Play these bags without `--clock`.** The workspace
README recommends `--clock`; that is harmless for VINS and ORB-SLAM3 because
neither consumes TF, and fatal here.

**An odometry TOPIC starves RTAB-Map's synchroniser.** Subscribing to odometry
puts five topics in one `ApproximateTime` set: odometry at 40 Hz plus four
image/`camera_info` topics at ~3 Hz. The matching odometry message is evicted
before its stereo pair arrives, and RTAB-Map reports *"Did not receive data since
5 seconds"*. Measured on an identical ~45 s run:

| odometry source | starvation warnings | nodes | words | links |
|---|---|---|---|---|
| `odom` topic | 7 | 3 | 97 | 1 neighbour, 1 closure |
| **TF (`odom_frame_id`)** | **0** | **28** | **194** | **25 neighbour, 2 closure** |

Both front-ends already broadcast the transform (`vins_estimator.cpp:224–232`;
`orbslam3_node.cpp` `publish_tf`, default true), so TF is used. Only the four
image/`camera_info` topics remain in the sync and they share a timestamp exactly.
**Cost:** RTAB-Map no longer receives odometry covariance to weight graph edges.

**`topic_queue_size` defaults to 10 and is not the sync queue.** Separate from
`sync_queue_size`. At 40 Hz, ten messages is 0.25 s of history — shorter than the
gap between throttled stereo pairs. Raised to 200. (`queue_size` is also
deprecated in 0.23; it maps to `sync_queue_size` with a warning.)

**RTAB-Map's `imu` input is a RELATIVE topic name.** It silently resolves to the
bag's `/imu`, then blocks waiting for TF to the Gazebo IMU frame
(`ackermann_robot_001/...`) that nothing publishes. Remapped to a dead topic
`/rtabmap/imu_unused`; the front-end already fuses IMU and gravity-aligns.

**`visual_odometry:=false` is not a node parameter.** It is an argument of the
upstream `rtabmap_launch` wrapper. The `rtabmap` node always takes its pose
externally — *not launching an odometry node* is what puts VINS or ORB-SLAM3 in
that role. Passing it to the node does nothing.

**Republishing at full frame rate is transport-bound, not decode-bound.** JPEG
decode measured at **5.12 ms/frame** colour, 2.88 ms mono — 195 Hz of headroom.
But bgr8 at 1280×720 is 2.76 MB/frame, so 24 Hz is 66 MB/s through DDS, and the
shim could only sustain ~1–2 Hz. Since RTAB-Map only checks for loops at
`Rtabmap/DetectionRate` (1 Hz), the shim throttles to 4 Hz on the *message stamp*
(so the kept subset is identical at any playback rate), measured at 3.23 Hz
effective — 3× margin over what is consumed.

**`ros2 topic hz` lies about large messages.** It reported 1.3 Hz for
`image_rect` and 2.5 Hz for `camera_info` — topics published in the *same*
callback. A dedicated counting subscriber showed 35 and 35. Do not trust
`topic hz` on multi-MB topics.

**The map came out TILTED because the graph had no gravity constraints.** The
occupancy grid sat at an angle to the horizontal and to the trajectory, and the
cloud did not project onto it. `Optimizer/GravitySigma` is 0.3 by default,
meaning "optimise while PRESERVING gravity orientation" — but it can only do that
if the graph *has* gravity links, and it had none: the IMU input is remapped away
(see below), and `Mem/UseOdomGravity` is **false** upstream. With no orientation
reference at all, loop-closure optimisation was free to rotate the entire map in
roll and pitch.

`Mem/UseOdomGravity: true` is the setting for this exact configuration — *"use
odometry instead of IMU orientation to add gravity links... we assume that
odometry is already aligned with gravity (e.g. VIO)"*. VINS is stereo-inertial
and gravity-aligns during IMU init, so its roll/pitch are absolute.

Measured on `dataset_static`, same run otherwise:

| | Neighbor | GlobalClosure | Gravity |
|---|---|---|---|
| `Mem/UseOdomGravity` false | 27 | 0 | **0** |
| `Mem/UseOdomGravity` true | 32 | 2 | **33** (one per node) |

Two gotchas when checking this yourself. Link type **9 is `kGravity`**, not
Landmark (`Link.h`: 7 `kPosePrior`, 8 `kLandmark`, 9 `kGravity`). And
`GravitySigma` is only 0.3 when the core is built with g2o or GTSAM, else 0.0 —
apt's `librtabmap_core.so` links both, so it is 0.3. Check the real library at
`/opt/ros/humble/lib/x86_64-linux-gnu/`, not the symlink directory.

**cv2.FileStorage SILENTLY MISREAD the camera extrinsics, and the shim published
a garbage rotation for days.** `config/wil_sim/stereo_imu.yaml` declares
`body_T_cam0` as `dt: d` but writes the whole numbers bare -- `0`, `1`, `-1`
instead of `0.0`, `1.0`, `-1.0`. OpenCV 4.11's YAML reader stuffs those into the
double array as raw ints, so **only the entries written with a decimal point come
back correct**. The translation column (0.158, 0.0936, 0.2475) read perfectly
while the entire rotation block came back as ~1e9 to 1e18:

```
[[ 4.2950e+09  1.1038e+12  7.0835e+18  1.5800e-01]
 [ 8.5899e+09  4.2950e+09 -1.7414e+18  9.3600e-02]
 [-1.0952e+12  8.5899e+09 -2.2136e+18  2.4750e-01]]      det = 2.16e+42
```

The failure is invisible without a check. The shim starts cleanly, prints the
right baseline (93.6 mm, since that comes from the translations), and publishes a
static TF whose quaternion tf2 normalises to a camera **pitched ~34 deg and yawed
~-27 deg** from where it actually is. Every projected point lands in the wrong
place and the occupancy grid comes out as smears instead of walls.

Fixed by parsing the `data: [...]` list from the text with a regex and running
every matrix through `_check_rigid()` -- orthonormal, `det == +1`, exit otherwise.
Verified against the robot SDF's own frame chain
(`base_footprint -> base_link -> cam_link1 -> cam_link2 -> cam0 -> cam0_undis`):
translation agrees to **5.6e-17**, rotation to **3.7e-06**, and the live
`/tf_static` is again `(0.5, -0.5, 0.5, -0.5)`.

**Lesson: validate every transform read from a file.** A rigid transform has a
cheap invariant and nothing else in the pipeline checks it.

**RTAB-Map's outputs are at ROOT, not under `/rtabmap`.** The node is named
`rtabmap` but sits in no namespace, and its publishers use relative names — so
the optimised graph is **`/mapPath`**, not `/rtabmap/mapPath`. Only the topics we
remap explicitly (e.g. `/rtabmap/imu_unused`) carry the prefix.

**`/mapPath` is published from two code paths and only one is usable.**
`CoreWrapper.cpp:4879`, the live per-iteration publish, stamps *every* pose with
the same message time — a CSV built from it has one timestamp repeated on all 46
rows and cannot be matched against ground truth at all. `CoreWrapper.cpp:4266`,
reached only through the `publish_map` service, stamps each pose with its node's
real time. `rtab_trajectory_writer.py` therefore calls
`/rtabmap/publish_map` (`global_map`, `optimized`, `graph_only`) on a 3 s timer
and once more at shutdown, and ignores any Path whose stamps are uniform.

**`database_path` pointed at a directory is a delete waiting to happen.** The
node runs with `-d`, which erases whatever `database_path` names before starting.
Because `database_path` and `output_path` take similar-looking values, it is
natural to pass the same run directory to both — and that aims a delete at the
directory holding the results. The launch file now resolves any value that is not
a `.db` file to `<that path>/rtabmap.db`, so `-d` can only ever remove the
database. A `delete_db:=false` argument was added as well.

(On 2026-09-04 the 16 `output_vins_rtab/simulation/*/vio.csv` files did go
missing right after such a launch, but they were recovered intact from
`~/.local/share/Trash` — and a *trash move* is not something `-d` can do, since
RTAB-Map calls `remove()` directly. The guard was added regardless: the hazard is
real even though this particular loss was not caused by it.)

**rclpy rejects `numpy.reshape(-1)` for `Image.data`.** Asserts "must be a set or
sequence"; `.tobytes()` is required. Not a bottleneck anyway.

---

## 6b. Stereo depth quality

The dense cloud came out as vertical smears — mismatches strung along the viewing
ray. Two causes, one fixable and one not.

**Not fixable: the baseline.** `fx = 640` with a 93.6 mm baseline gives
`fx*B = 59.9 px*m`, so a ONE-pixel disparity error costs `z^2 / 59.9`:

| range | error from 1 px |
|---|---|
| 3 m | 0.15 m |
| 5 m | 0.42 m |
| 10 m | **1.67 m** |

Past a few metres the points are rays, not measurements. `Grid/RangeMax` is set
to **4.0** (upstream default 5.0) and exposed as `grid_range_max:=`.

**Fixable: the matcher.** `Stereo/DenseStrategy` defaults to `0` = `cv::StereoBM`,
which needs local texture in every block — and a painted warehouse floor and blank
walls have none, so BM emits confident garbage. Set to `1` = `cv::StereoSGBM`.
`Grid/NoiseFilteringRadius` is also 0.0 (disabled) upstream; set to 0.1 with
`MinNeighbors` 5.

Measured on `dataset_dynamic_000`, assembled `/cloud_map`:

| | points | z p50 | z p99 | z max | above 1.5 m |
|---|---|---|---|---|---|
| BM, no filter, RangeMax 5 | 96727 | 3.79 m | 8.37 m | 9.41 m | **95.4 %** |
| SGBM, filter 0.1/5, RangeMax 4 | 86897 | **1.99 m** | **5.07 m** | **5.83 m** | **74.1 %** |

Median point height halves and the 99th percentile drops 3.3 m while only 10 % of
points are lost — the cloud is better structured, not merely cropped. A median
height of 3.79 m is obviously wrong for a floor-level camera at 0.2475 m.

**Caveat: this varied three settings at once and is not an ablation** — how much
comes from SGBM versus the range cap versus the noise filter is unmeasured.
74 % of points still sit above 1.5 m, so smears remain; `Grid/MaxObstacleHeight`
and a tighter `grid_range_max` are the next knobs.

### The floor produces no stereo depth at all

Chasing the noisy obstacle scatter turned up something more basic. RTAB-Map splits
ground from obstacles using point NORMALS by default (`Grid/NormalsSegmentation`,
45 deg cone), which on noisy stereo over a flat floor is meaningless. Switching to
a plain height threshold made the real situation visible:

| segmentation | ground pts | obstacle pts |
|---|---|---|
| normals (upstream default) | 146 | 76400 |
| height threshold, MaxObstacleHeight 2.0 | **0** | **46573** |

Obstacle noise fell 39 %, but ground went to zero — the floor is not being
*misclassified*, it yields **no stereo points whatsoever**. A blank white warehouse
floor has no texture for SGBM to match. Consequence: the occupancy grid had
obstacles and no free space at all, which is not a usable map.

`Grid/RayTracing: true` (upstream default false) fixes that by marking cells
between the sensor and each occupied cell as free. Measured after: grid 381x725
with free 31610, occupied 10430, unknown 234185 — a free/occupied ratio of 3.0
where it had been ~0.

Settings that ended up in the launch: `Grid/NormalsSegmentation false`,
`Grid/MaxGroundHeight 0.1`, `Grid/MaxObstacleHeight 2.0`, `Grid/RayTracing true`.

**Where this bottoms out.** The remaining sparseness is the rig, not the tuning:
a 93.6 mm baseline over untextured surfaces cannot produce dense geometry, and no
parameter recovers depth that was never measured. Further effort here has poor
returns.

**None of this affects ATE.** Loop closures come from visual words, not the dense
cloud. This tuning changes how the map looks, not how the trajectory is corrected.


---

## 6c. Why the occupancy grid is not navigable

The grid shows fan-shaped smears radiating from the trajectory rather than closed
walls and open aisles. Chased to root cause on 2026-09-07.

### What is NOT wrong

- **Trajectory.** VINS ATE 0.248 m on `dataset_static_tiny_repeat`.
- **Rectification.** Vertical disparity between ORB matches on a stereo pair:
  median **0.00 px**, 90 % within 1 px. The shim's calibration and left/right
  assignment are right.
- **Gravity / plane orientation.** Fixed earlier by `Mem/UseOdomGravity` (§6).

### Correction: `/map` and `/grid_prob_map` are the SAME state

An earlier note in this log suggested subscribing to `/grid_prob_map` for a
"more Bayesian" map that would suppress inconsistent observations. **That is
wrong.** Both topics render one `rtabmap::OccupancyGrid` instance
(`MapsManager.cpp:75`, `:649`, `:1440-1455`): `getMap()` thresholds it to ternary
-1/0/100 at `GridGlobal/OccupancyThr`, `getProbMap()` renders the *same cells* as
0..100 probability. Switching topics changes the rendering, never the estimate.
Any fix has to act on the log-odds accumulation itself.

### The integration rule is wrong for this sensor

`GlobalMap.h:45-54` accumulates log-odds. With the defaults (`Parameters.h:914-918`,
confirmed present in the recorded db):

| | probability | log-odds |
|---|---|---|
| `GridGlobal/ProbHit` | 0.7 | **+0.847** |
| `GridGlobal/ProbMiss` | 0.4 | -0.405 |
| `GridGlobal/ProbClampingMax` | 0.971 | +3.51 |
| `GridGlobal/OccupancyThr` | 0.5 | threshold **0.0** |

So **one hit marks a cell permanently occupied** (0.847 > 0), and a saturated cell
needs **9 consecutive misses** to clear. Against depth that disagrees with itself
40 % of the time at a 0.12 s baseline (§6b cross-view test), that is precisely the
wrong rule. `GridGlobal/OccupancyThr` is the only multi-observation gate:
net hits required = `ceil(logodds(thr)/0.847)` -> 0.5:1, 0.7:2, 0.9:3, 0.95:4.

### Re-assembling a stored database is ~50 ms, with no bag replay

The single most useful workflow fact found. Per-node local grids are computed once
at node insertion and stored (`MapsManager.cpp:549`, `:556-565`), so `Grid/*`
changes do NOT retroactively affect mapped nodes -- but `GridGlobal/*` acts at
**assembly** time and does. `rtabmap-export` accepts arbitrary parameter overrides:

```bash
rtabmap-export --map --opt 2 --GridGlobal/OccupancyThr 0.9 \
    --output_dir <dir> --output <name> <db>
```

50 ms per config, ~120 KB out, and the graph is byte-identical across configs by
construction -- which structurally fixes the "runs processed different node counts"
problem that invalidated an earlier A/B. `--opt 2` returns a CACHED grid if
`Admin.opt_map` is non-NULL, so always `md5sum` the outputs; all-identical means
that trap, not a null result. `rtabmap-reprocess` (regenerates local grids, so
`Grid/*` and `Stereo*/*` also need no replay) and `rtabmap-cleanupLocalGrids` are
installed alongside it.

### Measurement 1 -- raising the gate erases everything

26-node db, 8 loop closures, current settings, `GridGlobal/OccupancyThr` swept alone:

| thr | net hits needed | occupied cells | % of baseline |
|---|---|---|---|
| 0.50 (default) | 1 | 8338 | 100 % |
| 0.60 | 1 | 5709 | 68.5 % |
| 0.70 | 2 | 2034 | 24.4 % |
| 0.80 | 2 | 102 | 1.2 % |
| 0.90 | 3 | 1 | 0.0 % |
| 0.95 | 4 | 0 | 0 % |

### Measurement 2 -- ray tracing erases real structure too

"Net" hits combine hits and ray-traced misses, so a wall stereo fails to detect
from another angle gets actively cleared. Sweeping `ProbMiss` against `OccupancyThr`:

| | ProbMiss 0.40 (-0.405) | 0.45 (-0.201) | 0.48 (-0.080) |
|---|---|---|---|
| OccupancyThr 0.50 | 8338 | 9645 | 9752 |
| 0.70 | 2034 | 2506 | 2569 |
| 0.80 | 102 | 452 | **1608** |
| 0.90 | 1 | 18 | 124 |

At thr 0.80, weakening misses recovers 102 -> 1608 cells: ray-trace erasure is a
major contributor, not just non-repeatable hits.

### Measurement 3 -- the binding constraint is KEYFRAME DENSITY, not the sensor

The obvious reading of Measurements 1-2 is "the depth is unusable". That is too
quick. For each occupied cell, count the keyframes that could actually observe it
-- within `Grid/RangeMax` 4 m **and** inside the 90 deg horizontal FOV
(fx=640, width=1280 -> HFOV = 2*atan(640/640) = 90 deg):

| | ignoring FOV | **with FOV** |
|---|---|---|
| p10 | 6 | 2 |
| p50 | 7 | **3** |
| p90 | 9 | 4 |
| max | 12 | 6 |

| cells viewable by >= N keyframes | 1 | 2 | 3 | 5 |
|---|---|---|---|---|
| share | 99.2 % | 93.5 % | **69.8 %** | 3.4 % |

**A median cell is seen 3 times.** Demanding 3 net hits therefore requires perfect
detection from every single opportunity, before ray-trace misses subtract anything.
The multi-hit gate is not failing because the depth is hopeless -- it is failing
because there is almost nothing to integrate. `Rtabmap/DetectionRate` is **1.0 Hz**,
which produced 26 nodes for a full loop.

**Conclusion: raise keyframe density first, then gate on repeatability.** Tuning
`GridGlobal/*` alone cannot work at 3 observations per cell. Ignoring FOV in this
calculation gives 7 and the opposite (wrong) conclusion -- worth remembering.

### RETRACTED (2026-09-07): Measurements 1, 2 and 4 were built on a broken TF

Everything below that scored an RTAB-Map GRID was produced while the shim was
publishing the garbage camera rotation described in §6 -- the extrinsics parse
bug. The maps were built with the camera ~34 deg off in pitch, so the sweeps, the
precision/recall table and the "not reachable by tuning" verdict **do not hold**
and must be re-run against a corrected shim.

What still stands, because it never used the shim:
  * the cross-view consistency test in §6b (IoU 0.611 at 0.12 s and the decay with
    viewpoint separation) -- that harness hardcoded the correct `body_T_cam` and
    read poses straight from the bag;
  * the log-odds arithmetic, which is a property of RTAB-Map, not of this data;
  * the 50 ms `rtabmap-export` re-assembly workflow;
  * the keyframe counts (59 at 1 Hz, 162 at 4 Hz) and the 464 ms/keyframe cost;
  * the FOV argument in Measurement 3 -- the geometry is right, but the cell
    positions it was evaluated on came from a broken map, so the median-3-views
    number needs recomputing.

The baked ground-truth map and the comparison tooling are unaffected and ready to
re-use.

### Measurement 4 -- more keyframes, and the verdict

`Rtabmap/DetectionRate` is now a launch argument. Re-ran `dataset_static_tiny_repeat`
end to end with `script/gt_tf_publisher.py` (new -- broadcasts ground_truth.csv as
`world -> body`, so the map is isolated from odometry drift and the map frame IS the
Gazebo world frame, making comparison identity-aligned).

RTAB-Map needs **464 ms per keyframe** (SGBM at 1280x720), so the bag must be played
at `--rate 0.5`; at `--rate 2.0` it silently dropped to 15 nodes with 10.5 s of delay.

| DetectionRate | nodes | loop closures |
|---|---|---|
| 1.0 Hz | 59 | 9 |
| 4.0 Hz | **162** | 9 |

Density does lift what survives the gate -- at 3 net hits, 2092 cells survive with
162 keyframes vs 305 with 59 and **1** with the original 26:

| gate | 59 keyframes | 162 keyframes |
|---|---|---|
| 1 hit | 10266 (100 %) | 12384 (100 %) |
| 2 hits | 1196 (11.7 %) | 4110 (33.2 %) |
| 3 hits | 305 (3.0 %) | 2092 (16.9 %) |

**But surviving is not the same as being right.** Baked the ground-truth grid with
`script/bake_map.py --world .../small_warehouse_static_tinymap.world` (its defaults
are already sized for this building: res 0.05, origin (-7.0,-10.5), 286x423, seed
(1.8,9.0)) and scored the estimate against it at 162 keyframes:

| gate | precision @0.25 m | recall @0.25 m | F1 | chamfer p95 |
|---|---|---|---|---|
| 1 hit | 42.2 % | **34.8 %** | **38.1** | 1.85 m |
| 2 hits | 41.8 % | 18.1 % | 25.2 | 2.14 m |
| 3 hits | 45.4 % | 12.1 % | 19.1 | 2.25 m |

**Precision is FLAT while recall collapses.** A cell observed three times is no more
likely to be real than one observed once -- the gate removes true and false cells at
the same rate, so repeatability does not correlate with correctness here. The loosest
gate is the best one, and even it leaves ~58 % of mapped obstacles phantom at a 25 cm
tolerance with a chamfer p95 of 1.85 m (i.e. smears radiating into open floor, not a
thin blur around real walls).

**Verdict: the occupancy grid is not reachable by tuning the integration rule.**
This is the textbook signature of "the repeatable subset is empty" -- every setting
that removes the fog removes the walls with it. `GridGlobal/*` tuning, keyframe
density, ray-trace strength and the multi-hit gate have all been swept; none moves
precision. What remains is either a different sensor model (single-scanline
`depthimage_to_laserscan` + `Grid/Sensor=0`, installed but untried) or accepting the
map as a qualitative figure.

**None of this affects the ATE results.** Loop closures come from visual words, not
the grid. The trajectory comparison stands on its own.


### Measurement 5 -- re-run with the TF fixed. The bug WAS the problem.

Same pipeline, same dataset, same ground-truth odometry, only the extrinsics
parse corrected. `dataset_static_tiny_repeat`, `detection_rate:=4.0`,
bag at `--rate 0.5`.

**The graph changed first.** Loop-closure verification is geometric, so a 34 deg
camera error was crippling it:

| | nodes | loop closures | local-space links |
|---|---|---|---|
| broken TF | 162 | 9 | 2 |
| **fixed TF** | 164 | **72** | **17** |

**Then the map.** Scored against the baked ground truth at tau = 0.25 m:

| | occupied cells | precision | recall | F1 | chamfer p95 |
|---|---|---|---|---|---|
| broken TF | 12384 | 42.2 % | 34.8 % | 38.1 | **1.85 m** |
| **fixed TF** | **4770** | **90.8 %** | **42.0 %** | **57.4** | **0.35 m** |

Precision more than doubled, the phantom count fell by 62 %, and chamfer p95 went
from 1.85 m (smears radiating into open floor) to 0.35 m (a thin blur hugging real
walls). Visually the oval of red phantom obstacles is gone and the shelf faces and
west wall come out as continuous blue lines with visible aisles between them --
see `output/compare/rtab_grid_tf_fix.png`.

**What this overturns.** Every earlier conclusion about grid quality was measured
through the broken transform:
  * "the depth is not repeatable enough to build a map" -- unsupported;
  * "precision is flat while recall collapses, so the repeatable subset is empty"
    -- an artefact; with the TF fixed, precision at the loosest gate is 90.8 %;
  * the `Grid/*` tuning (`grid_range_max`, `NormalsSegmentation`, `MaxGroundHeight`,
    `RayTracing`, SGBM strategy) was all chosen to suppress smears that were
    actually a frame error. Those settings should be re-derived from scratch.

**What still holds.** `GridGlobal/OccupancyThr` is still not the lever, but now for
the opposite reason: precision is already 90.8 % at 1 net hit, so tightening the
gate only costs recall (42.0 % -> 10.2 % at 3 hits) for ~4 points of precision.
Leave it at the default 0.5.

**The real remaining limit is RECALL, not precision.** 42 % means over half the
observable geometry is still missing, which points at coverage: `grid_range_max`
is 4.0 m while the measured depth distribution has its p75 between 4.7 and 10.5 m
(§6b). That is the next thing to sweep, and now it can be swept against a map that
is not lying.


### Measurement 6 -- re-deriving Grid/RangeMax with a correct TF

`rtabmap-reprocess` regenerates local grids from a stored db (verified: `Grid/CellSize`
0.05 -> 0.20 propagated, same 164 nodes), so `Grid/*` can be swept offline with no
bag replay. Swept `Grid/RangeMax` on `dr4.0.db`, scored against the baked truth at
tau = 0.25 m:

| range | cells | precision | recall | F1 | chamfer p95 |
|---|---|---|---|---|---|
| 4 m | 4762 | **94.0 %** | 44.8 % | 60.6 | 0.30 m |
| 6 m | 8974 | 90.0 % | 41.5 % | 56.8 | 0.35 m |
| 8 m | ~11000 | 86.0 % | 49.8 % | 63.1 | 0.42 m |
| **10 m** | 8698 | 89.4 % | 50.9 % | **64.9** | 0.35 m |
| 12 m | 11275 | 87.9 % | **52.2 %** | 65.5 | 0.40 m |

Recall rises 45 % -> 51 % from 4 m to 10 m while precision only falls 94 % -> 89 %
and chamfer p95 is flat. **The far points are mostly real.** The 4.0 m cap was set
to crop smears that were actually the camera-TF bug (fix 13), and it was costing
coverage for nothing. Default changed to **10.0**.

**`rtabmap-reprocess` is NOT deterministic.** It re-runs loop detection: 129 vs 133
closures on identical input, and 8 m first scored F1 37.8 then 63.2 and 62.9 on
repeat -- the first was simply a failed run. Repeat noise is about 1 F1 point, so
never trust a single run or a non-monotonic trend. (It also finds far MORE closures
offline than the live recording did -- 129-133 vs 72 -- so reprocess numbers are an
upper bound on what runs live.)


### Measurement 7 -- live-pipeline node counts vary ~30 % run to run

Adding `mode:={construct|extend|localize}` to `rtab_sim.launch.py` had to be proved
backwards compatible. The outcome test disagreed with the baseline:

| | nodes | closures |
|---|---|---|
| baseline (pre-mode launch) | 164 | 72 |
| `mode:=construct` | 117 | 45 |

That is a 29 % gap, and it is **not** the change. `ros2 param dump /rtabmap` under
each launch file is **byte-identical** (15957 bytes, empty diff), so the node
receives exactly the same parameters either way.

The variance is frame drops under machine load: the same configuration was already
measured at 15 nodes when the bag played at `--rate 2.0` versus 162 at `--rate 0.5`.
The pipeline is timing-sensitive, and background work on the machine is enough to
move it by a third.

**Consequence for any future A/B: compare the parameter dump, not the node count.**
An outcome difference under ~30 % on a live run carries no information. This is the
live-pipeline analogue of the `rtabmap-reprocess` non-determinism in Measurement 6
(129 vs 133 closures on identical input) -- the offline path is ~1 F1 point of noise,
the live path is ~30 % of the node count, which is far worse. Pin the comparison at
the parameter level wherever possible, or replay the same database offline.


---

## 6d. Localize / construct / extend modes

RTAB-Map driven from RViz+nav2 as a localizer, with odometry from a selectable
front-end and **no ground truth**. viewer.py stays read-only about localization.

### Frames -- rename at the source, one static bridge

VINS publishes `world -> body`; ORB-SLAM3 publishes `orbslam3_world ->
base_footprint`; nav2 and rtabmap both want `map -> odom -> <robot>`. Both
front-ends expose `world_frame_id` / `body_frame_id` as ROS parameters
(`vins_estimator.cpp:59-60`, `orbslam3_node.cpp:96-97`), so both are told to publish
**`odom -> vio_body`** and everything downstream is front-end independent.
`frontend:=` then changes exactly one thing: which executable runs.

    map  --(20 Hz)------------->  odom            rtabmap (the correction)
    odom --(front-end rate)---->  vio_body        VINS *or* ORB-SLAM3, never both
    vio_body --(static)-------->  base_footprint  localization_rtabmap.launch.py
    base_footprint --> base_link --> sensors      robot_state_publisher
    vio_body --> cam{0,1}_optical                 rtab_camera_shim.py

**ORB-SLAM3's `body_frame_id` default is WRONG, and this fixes it.** It defaults to
`base_footprint`, but `slam_wrapper.cpp:207` computes `T_w_b = T_w_c * T_b_c0^-1`
with `T_b_c0` from the config's `IMU.T_b_c1` -- so the pose published is the IMU
body. Under its own default ORB places the robot **0.338 m forward and 0.192 m
high**.

The bridge constant comes from `model.sdf`: `imu_link = base_footprint +
(0.338, 0, 0.192)`, zero rpy. Cross-checked independently -- the URDF chain
`base_footprint -> cam0` minus `base_footprint -> imu_link` equals
`body_T_cam0`'s `(0.158, 0.0936, 0.2475)` **exactly**.

### The free runtime assertion

Because the URDF reaches the camera through `base_footprint` and the shim reaches it
through `vio_body` + the calibration file, the two paths must meet:

    ros2 run tf2_ros tf2_echo cam0_undis cam0_optical     # MUST be identity

**Measured: translation (0,0,0), quaternion (0,0,0,1).** This is a continuous check
on the entire chain -- extrinsics file, static bridge, URDF and shim at once -- and
it is exactly the check that would have caught fix 13 in seconds.

Full chain verified with a bag and `start_frontend:=False`:
`map->odom` identity (no closure yet), `odom->vio_body` (-1.305, -4.487, 0.021),
`vio_body->base_footprint` (-0.338, 0, -0.192), `map->base_link` resolves.

### Modes

`rtab_sim.launch.py mode:={construct|extend|localize}`. The ~6 differing parameters
are a dict merge (`_mode_params`), not a forked file, because the other ~24 carry the
measured comment blocks that are this file's real value.

| | construct | extend | localize |
|---|---|---|---|
| `-d` | yes | no | no |
| `Mem/IncrementalMemory` | true | true | **false** |
| `Mem/InitWMWithAllNodes` | false | **true** | **true** |
| extras | | | `RGBD/MaxOdomCacheSize` 10, `RGBD/StartAtOrigin` false, `use_saved_map`, `loc_thr` 0.2 |

`Mem/InitWMWithAllNodes` matters most in `extend`: without it working memory holds
only the *previous* session, so a third session could attach to the second but never
to the first.

`delete_db` now defaults to empty = "the mode decides". `delete_db:=true` on
extend/localize is **refused**, because `-d` is purely a file delete and would erase
the map those modes exist to read. There is no `save_database` service -- the DB is
written on clean shutdown -- so always `cp` before `extend`.

**Backwards compatibility proven at the parameter level**, not the outcome level:
`ros2 param dump /rtabmap` under the old and new launch files is byte-identical. See
Measurement 7 for why the outcome level could not settle it.


### Levels 1-2 verified with VINS and NO ground truth

First end-to-end runs of the localization stack with a real front-end.
`dataset_static_tiny_repeat`, `--rate 0.5`, `detection_rate:=1.0`.

**Two blockers found, neither of them in the new code.**

**(a) VINS cannot consume the recorded bags at all.** It subscribes to raw
`sensor_msgs/Image` and has no image_transport support (grep `CompressedImage` in
`vins_estimator.cpp`: nothing), while the bags carry only
`/camN/image_raw/compressed`. It has always worked against the LIVE sim, which
bridges raw when `compressed_images:=False`, so the gap had never been exercised.
The symptom was indirect and misleading: rtabmap logged 12 `getTransform()
odom -> base_footprint` failures and built **0 nodes** -- nothing said "VINS has no
images". Fixed with an explicit `decompress:=true` argument that republishes
compressed->raw, rather than repointing VINS at the shim's already-decompressed
`/stereo/*/image_rect`: that would change VINS' input relative to the recorded
baselines. ORB-SLAM3 needs none of this.

**(b) `RGBD/OptimizeMaxError` 3.0 rejects EVERY loop closure a drifting VIO
proposes.** This is the substantive finding.

| `RGBD/OptimizeMaxError` | nodes | loop closures | rejected |
|---|---|---|---|
| 3.0 (upstream default) | 52 | **0** | 26 |
| **10.0** | 47 | **36** | 6 |

Appearance-based place recognition works either way -- 36 closures were *detected*
in both runs. What differs is geometric verification: the guard rejects a closure
whose implied correction is too large, and VINS drift over a warehouse lap makes
every correction too large at 3.0 (`Rtabmap.cpp:4033 Rejecting all added loop
closures`). Upstream's own multisession demo raises it to 10 for the same reason.

**This reframes "RTAB-Map improves VINS": the benefit is gated by a threshold that
must be loosened for a drifting front-end, and loosening it accepts larger
corrections AND larger mistakes.** Both numbers belong in the write-up. The earlier
GT-driven measurements hid this entirely, because a perfect trajectory never trips
the guard -- it accepted 72 closures on the same bag.

`localization_rtabmap.launch.py` defaults it to 10.0 (the VIO path);
`rtab_sim.launch.py` keeps upstream's 3.0 so the bare mapping tool is stock.

**Level 2 -- localize against that map, read_only:**

| | |
|---|---|
| localized samples | **42/43 (98 %)** |
| first fix | 1.8 s |
| map->odom at start | exactly identity |
| steady-state step | median 0.057 m, max 0.535 m |
| teleports (>1 m) after acquisition | **0** |
| database md5 before/after | **identical** -- read_only works |

The two >1 m steps are both inside the first 3 s and are the expected acquisition
transient: `RGBD/StartAtOrigin:=false` starts from the previous session's last
localization pose (10.46 m away, at the end of the lap) and it corrected to the
truth within one update. Correction then grows smoothly to ~0.6 m over 54 s, which
is VINS drift being tracked.

**Not yet established:** that the 36 accepted closures are *correct*. A looser guard
accepts more mistakes too, and a warehouse of near-identical bays is the worst case.
The direct test is the aliasing stress (localize against a different area; correct
behaviour is to REFUSE), which has not been run.

**VINS throughput at `--rate 0.5`: backlog peaked at 9, ended at 0.** Comfortable for
the bag path; the live-sim path remains unmeasured.


### Level 3 -- and the trap in "98 % localized"

**Covariance < 9999 is NOT a correctness signal.** It means "I matched a place",
not "this pose is right". Level 3B reported **98 % localized, 26/26 on the revisit
lap** -- and every one of those numbers was worthless:

| | |
|---|---|
| `/localization_pose` extent | x [0.3, **161.9**], y [**-387.5**, 0.0] |
| the tiny warehouse | 14 x 21 m |
| `map->odom` throughout | 0.80 m |

RTAB-Map was not correcting anything. It was passing a **diverged VINS** straight
through and reporting confidence about it. A pass criterion built on the covariance
flag alone will accept exactly this. **Any future criterion needs a sanity bound on
the pose itself.**

**Root cause: resource contention, proven by elimination.**

| hypothesis | test | result |
|---|---|---|
| `republish` starved VINS | when did it die? | line 116673 of 116674 -- my own teardown `pkill`. Ran the whole time |
| `republish` rewrites stamps | compare stamps on both topics | **221 of 221 identical**. Preserved |
| the decompress path is bad | **run VINS ALONE on the same bag+republish** | x [0, 10.67], y [-0.17, 0], max step 0.26 m -- **no divergence** |

So VINS is fine on this path by itself and diverges only with rtabmap + shim +
republish running alongside. Its image backlog stayed at max 9, so it was not
starved of *images* -- the likely mechanism is dropped `/imu` samples (200 Hz over
DDS under load), which wrecks preintegration. Recorded VINS baseline on this bag for
comparison: x [0.0, 13.3], y [-5.7, 0.2].

**This is the plan's live-sim throughput uncertainty arriving early, on the bag
path.**

**Pacing is NOT the fix -- tested and refuted.** Re-running at `--rate 0.3`:

| | `--rate 0.5` | `--rate 0.3` |
|---|---|---|
| localized AND sane | 0 % | **0 %** |
| pose extent \|x\| | 161.9 m | **1595.7 m** (worse) |
| `/imu` median | not measured | **51 Hz vs 60 nominal** |

Slower playback diverged VINS *further*, and the IMU hypothesis is confirmed at the
same time: ~15 % of IMU samples are lost under load. The problem is total CPU, not
the rate at which work arrives.

**The targeted fix: stop rtabmap doing work it throws away.**
`RGBD/CreateOccupancyGrid` is **false upstream**, but the ROS wrapper forces it true
(`CoreNode.cpp:45`, `CoreWrapper.cpp:339`) because a mapping node needs it. In
LOCALIZATION mode `/map` is served from the database (`use_saved_map`), so every
local grid built is discarded -- and building one runs dense SGBM on the stereo
pair, the measured 464 ms/keyframe. That is the load starving VINS. Now set false
in `_mode_params('localize')`. Feature extraction for loop closure is untouched, so
relocalization is unaffected.

**Verified: it helps materially but does NOT solve the problem.**

| | grid ON | grid OFF |
|---|---|---|
| localized AND sane | 0 % | **24 %** (12/49) |
| pose extent \|x\| | 161.9 m | 225.7 m |
| `/imu` median | 51 of 60 (85 %) | 84 of 100 (84 %) |

Keep the setting -- discarded work is worth removing regardless -- but VINS still
diverges, so co-running VINS and rtabmap from a bag on this machine remains
unviable.

**The IMU mechanism is WITHDRAWN.** Proportional loss is ~15 % with the load fix and
~15 % without it, i.e. unchanged by removing the dominant CPU cost. And the recorder
subscribes BEST_EFFORT to `/imu`, so the figure measures the RECORDER's drops, not
what VINS receives. It was never evidence about VINS. The divergence mechanism is
unidentified.

### The way round it: replay recorded odometry

`script/gt_tf_publisher.py` already takes a `csv` parameter and reads the 11-column
`t_ns,x,y,z,qw,qx,qy,qz,vx,vy,vz` format -- which is exactly the format of the
recorded `vio.csv` files under `output/output_vins/`. So it can replay a REAL VINS
trajectory as `odom -> vio_body` at near-zero CPU:

    python3 script/gt_tf_publisher.py --ros-args -p use_sim_time:=true \
        -p csv:=output/output_vins/simulation/dataset_static_tiny_repeat/vio.csv \
        -p world_frame:=odom -p body_frame:=vio_body

    ros2 launch .../localization_rtabmap.launch.py start_frontend:=False mode:=localize ...

This is NOT ground truth -- it is VINS' own output, drift included -- so it tests the
localization stack against a realistic trajectory while removing the contention
entirely. It is also deterministic, so A/Bs mean something. The live co-execution
question stays open, but every Level 2-4 criterion can be measured this way first.

**`--start-offset` is unusable with VINS.** The plan's split-bag method crashes it
(exit -11, after logging `Re-init`): starting a VIO mid-trajectory hands it an IMU
stream with no preceding history. Use the repeat-loop structure instead -- build
from lap 1, localize over the full bag, and lap 2 is the genuine revisit.

**Status: Level 3 NOT passed.** Level 1 and Level 2 stand (Level 2's poses were
warehouse-scale and `map->odom` behaved, checked after the fact). The aliasing
stress 3(f) has not been run, so `optimize_max_error:=10.0` remains **effective but
not shown safe**.


---

## 7. Current state

### Localization modes -- what is built and what is proven (2026-09-07)

| # | Component | State |
|---|---|---|
| 1 | `script/viewer.py` localization readout | **done, screenshot-verified** in both states. `--gt-topic ''` no longer crashes; `--align none` no longer a silent no-op |
| 2 | `rtab_sim.launch.py` `mode:={construct,extend,localize}` | **done.** Backwards compatibility proven at the PARAMETER level (`ros2 param dump` byte-identical), not the outcome level |
| 3 | `localization_rtabmap.launch.py` + reaper group | **done, TF chain verified.** `tf2_echo cam0_undis cam0_optical` returns exact identity |
| 4 | Level 1 construct, Level 2 localize | **pass** (see Levels 1-2 section) |
| 5 | Level 3 relocalization / aliasing | **BLOCKED** -- VINS diverges when co-run with rtabmap. Not a localization-design failure |
| 6 | nav2 local-costmap fix | not started |
| 7 | `nav2_rtabmap.launch.py` + RViz config | not started |
| 8 | `extend` mode, Level 4 | not started |

**New files:** `script/gt_tf_publisher.py` (153 lines),
`aws-robomaker-small-warehouse-world/launch/localization_rtabmap.launch.py` (295).
**Modified:** `script/viewer.py`, `rtabmap_ros/rtab_sim.launch.py`,
`.../launch/stale_process_reaper.py` (localization group now covers the front-ends,
the static bridge and the shim).

**Maps kept:** `map/rtabmap_tiny_vins.db` -- 47 nodes, **36 loop closures**, built by
VINS with no ground truth at `optimize_max_error:=10.0`. This is the first map in the
project built end-to-end without ground truth.

### The three findings that matter most

1. **`RGBD/OptimizeMaxError` 3.0 rejects every loop closure a drifting VIO
   proposes.** 36 detected, 0 accepted; at 10.0, all 36 accepted. Ground truth
   accepted 72 on the same bag, which is why every earlier measurement hid this.
   "RTAB-Map adds loop closure to VINS" is true only with a threshold loosened for
   drift -- and loosening it accepts larger mistakes too. **Both numbers belong in
   the write-up.**
2. **Covariance < 9999 is not a correctness signal.** A run reported 98 % localized
   with poses at 161 m in a 14x21 m building. Any criterion needs a pose sanity
   bound, which `loc_recorder.py` now applies.
3. **VINS and rtabmap cannot co-run on this machine from a bag.** Pacing does not
   fix it (`--rate 0.3` was worse). Disabling discarded grid work helped (0 % ->
   24 % sane) but did not solve it. Mechanism unidentified -- the IMU theory was
   withdrawn as unsupported.

### Next step when resuming

Use the replay route (see "The way round it" above): `gt_tf_publisher.py` fed the
RECORDED `vio.csv` replays a real VINS trajectory as `odom -> vio_body` at near-zero
CPU, deterministically. Every Level 2-4 criterion including the aliasing stress can
be measured that way, and A/Bs finally mean something. Live co-execution stays an
open question, not a blocker for the evaluation.

---

### Mapping-mode files (earlier work)

Files (see §8 — their location is unresolved):

- **`rtab_camera_shim.py`** — decompresses the stereo pair, publishes
  `/stereo/{left,right}/image_rect` (+ `camera_info`) and static TF
  `body → cam0_optical, cam1_optical`, throttled to 4 Hz.
- **`rtab_sim.launch.py`** — the shim plus the `rtabmap` node. No odometry node.
- **`rtab_trajectory_writer.py`** — writes RTAB-Map's optimised graph as a
  `vio.csv` in the same 11-column format as `script/extract_gt.py` and both
  estimators. Started only when `output_path` is given. Poses are one per graph
  node (~0.6 Hz measured), **far sparser** than the front-ends' per-frame output,
  so any metric assuming equal row counts needs timestamp matching. Velocities
  are central differences between optimised poses, not filter state.

```bash
# terminal 1
ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py
# terminal 2
ros2 launch ./rtabmap_ros/rtab_sim.launch.py \
    output_path:=output/output_vins_rtab/simulation/dataset_dynamic_000
# terminal 3   (NO --clock)
ros2 bag play dataset/dataset_dynamic_000 --rate 0.5
```

ORB-SLAM3 instead: `odom_frame:=orbslam3_world body_frame:=base_footprint`.

`output_path` takes a directory (`vio.csv` is appended) or a path already ending
in `.csv`. Empty — the default — writes nothing, so exploratory runs cannot
silently overwrite a recorded result.

Datasets were renamed on 2026-09-04 to `dataset_{static,dynamic}_NNN`; the older
`static_dataset_loop` / `*_dataset_large*` bags referenced in earlier sections no
longer exist. Topic layout is unchanged: still no `camera_info`, still compressed
images, still a recorded `/clock`.

**Verified** twice, both with `/ground_truth/odometry` republished as the pose
source (a stub that also broadcasts `world -> body` TF, since RTAB-Map takes the
pose from TF):

| run | dataset | nodes | links | warnings |
|---|---|---|---|---|
| 2026-09-01 | `static_dataset_loop`, ~40 s | 28, 194 words | 25 neighbour, **2 closures** | 0 |
| 2026-09-04 | `dataset_dynamic_000`, ~70 s | 46 | 45 neighbour, **3 closures** | 0 |

The second run also produced a 42-row, 11-column `vio.csv` with 42 distinct
timestamps spanning 70.5 s, which `script/vio_metrics.py load()` parses without
modification.

That validation used **ground truth**, not VINS. It proves the plumbing, not the
result — with real VINS drift the closure count and the ATE benefit will differ,
which is the measurement the integration exists to make.

---

## 7b. Placing a database map in the 2D viewer (2026-09-09)

### The frames really are different, and here is the proof

The user's claim was correct. `script/viewer.py` draws in **Gazebo world
coordinates and nothing else**: obstacles and walls come from the `.world` file
(`load_obstacles`, viewer.py:199), ground truth arrives in world coordinates, and
estimator trails are pushed through `est.align` — a rigid 2D transform *solved
against ground truth* (viewer.py:724-733) — so they land in world coordinates too.
`View.to_screen` (viewer.py:1173) is a plain world→pixel affine with **no frame
parameter at all**.

An RTAB-Map occupancy grid is the one thing on screen that is not in that frame. It
arrives on `/map` in the `map` frame, anchored wherever the front-end initialised.

**RViz never has this problem because it does the opposite.** Its fixed frame is
`map`, so it moves the *robot* into the map frame via TF, rather than moving the map
into the world. It therefore needs no placement number and can never have a wrong
one. That asymmetry — not a bug — is the whole story.

### world→map is CONSTANT. Do not solve it live.

Two failed attempts (both reverted) tried to compute the placement at runtime:

1. **Kabsch fit of `/localization_pose` against ground truth.** Needs GT, and the
   9999 covariance means there is nothing to fit until the first relocalization.
2. **`map→odom` from TF composed with the estimator's `odom→world` alignment.**
   This is the one that looked most principled and was the worst in practice.

Attempt 2 fails for a reason worth writing down: **`map` and `world` are both static
frames, so `world→map` is a constant.** What moves is `odom` — `map→odom` *is* the
correction RTAB-Map applies as the front-end drifts. Composing it with an
`odom→world` alignment that was solved once, at `t=0`, and never revised means the
map wanders by exactly the accumulated drift. It is right at the start and gets
steadily worse, which is what the user saw.

So the correct answer is a constant, and the only real question is how to get the
right one instead of guessing.

### The measured answer, and why the guess was "almost right"

`script/fit_map_pose.py` (new) fits the grid's occupied cells to the world file's
obstacle and wall edges: brute force over yaw, one FFT cross-correlation per yaw for
the translation, scored against a blurred edge field. No odometry, no ground truth.

For `map/map_tiny.db`:

| | x | y | yaw |
|---|---|---|---|
| guess in use (`base_footprint` spawn) | 1.800 | 9.000 | −90.00 |
| **measured fit** | **1.850** | **8.600** | **−89.90** |
| predicted: spawn of the **IMU body** frame | 1.800 | 8.662 | −90.00 |

Margin over the best rival more than 5° away: **+60.9 %** — decisive, not a coin
toss in a warehouse of parallel aisles.

**The 0.40 m error was the body-vs-base_footprint offset.** VINS and ORB-SLAM3 both
report the **IMU body**, which sits 0.338 m ahead of `base_footprint`
(`BODY_TO_BASE` in `localization_rtabmap.launch.py`). At the spawn yaw of −90° that
0.338 m lies almost entirely along −y: 9.000 − 0.338 = 8.662. The fit lands **0.080
m** from that prediction — inside the grid's own 0.05 m resolution. Two independent
routes to the same number, which is what makes this a measurement rather than
another guess.

    python3 script/viewer.py --rotate 180 --world <world> --live-poses \
        --map ros --map-pose 1.85,8.60,-90

### Validation against ground truth (2026-09-09)

`script/validate_map.py` → `output/compare/rtab_map_vs_gt.png`. Two questions, two
different kinds of truth.

    python3 script/extract_gt.py \
        dataset/dataset_static_tiny_repeat/dataset_static_tiny_repeat_0.db3 gt.csv
    python3 script/validate_map.py map/map_tiny.db --world <tinymap.world> \
        --gt gt.csv --map-pose 1.85,8.60,-90

**1. Geometry, vs the `.world` file** (tolerance 0.25 m):

| | |
|---|---|
| precision | **96.6 %** of 11 634 occupied cells |
| recall, region the map observed | **89.5 %** |
| recall, whole building | 67.2 % |

Recall is measured against **surfaces**, not filled interiors — a stereo camera can
only see a shelf's outside, so counting its interior as missed geometry would cap a
perfect map well below 100 %. The gap between 89.5 % and 67.2 % is route coverage,
not mapping failure: the run never entered the side aisles.

**2. Drivability, vs the ground-truth trajectory.** The robot demonstrably drove
this route, so stamping the *real* nav2 footprint (a 0.83 × 0.50 m rectangle, not a
radius) onto the map at each of 2973 poses turns any collision into a **proven**
false obstacle:

| | |
|---|---|
| clear and mapped | **99.3 %** |
| false obstacle | **0.7 %** |
| unmapped under the footprint | 0.0 % |
| clearance, median / p05 | 1.07 m / 0.45 m |

**This is a much better map than §6c concluded**, and the reason is the corrected
placement: §6c's "not navigable" was measured with the grid mis-seated, so real
structure was being scored against the wrong cells.

**Three caveats that keep this honest:**

1. The ground truth is a **different run** (`dataset_static_tiny_repeat`) from the
   one that built `map_tiny.db` (built live; its node stamps match no dataset). The
   warehouse is static, so this is a valid geometry and drivability probe. It is
   **not** a trajectory-error measurement.
2. **The route is easy.** It is one large loop down the wide central aisle and never
   squeezes into a side aisle, so 0.7 % false obstacles is a floor, not a worst case.
   A route through the narrow aisles is the test that would actually stress this.
3. **p05 clearance 0.45 m is below nav2's `inflation_radius` of 0.80 m**, and a
   visible share of the true route sits inside it. Nothing is blocked, but a planner
   would charge a high cost for driving where the robot really drove. That coupling
   is the thing to look at before trusting a planned path.

Numbers are not comparable to §6b's ~89 % / ~51 %: different tolerance, and recall
there was not restricted to observed surfaces.

### Correction (2026-09-18): "node stamps match no dataset" is wrong

Caveat 1 above is half right. The ground truth **was** a different run from the one
that built the database -- that part stands. But the parenthetical claim that
`map_tiny.db`'s node stamps "match no dataset" is false, and it cost a day of
reasoning in the wrong direction.

All 30 nodes fall inside the sim-time interval of `dataset/dataset_map_tiny`, and
they are that run's trajectory. Applying the measured placement (1.85, 8.60, −90)
to the stored node poses puts them a median of **0.069 m** from that bag's
ground-truth route, and — the discriminating test — a median of **0.340 m**
(max 0.52 m) from the ground-truth pose carrying the *same* timestamp. A different
run over the same loop would agree in space but not in time. The wrong-sign yaw
gives 19 m, so the test does discriminate. The two routes are also visibly
different: `dataset_static_tiny_repeat` is a single oval circuit, `dataset_map_tiny`
a figure-eight through the centre.

**Why the original claim looked true.** A bag's `messages.timestamp` column is the
wall-clock *receive* time, while node stamps and message *header* stamps are
simulation time. Comparing the two gives an offset of ~1.8e9 s and the appearance
that nothing matches. Compare header stamps to node stamps, never row timestamps.

**Consequence.** `dataset_map_tiny` is not an equivalent substitute for the lost
ground-truth run. Re-running `validate_map.py` against it returns **100.0 % clear
and 0.0 % false obstacle** — the map cannot contradict the trajectory it was built
from — against 99.3 % / 0.7 % for the independent run. The 0.7 % is exactly what
only an unseen route can expose, and the 0.0 % must not be quoted as map quality.
Both runs are recorded in `output/compare/rtab_map_vs_gt_metrics.csv`, with the
second figure as `rtab_map_vs_gt_selfcheck.png`. Note that the evaluator prints
"Ground truth is a DIFFERENT run" unconditionally, so that footer is wrong on the
self-check figure.

### What a rigid placement cannot fix

Residual error in the overlay is RTAB-Map's own internal distortion — walls doubled
and thickened, shelf rows bowed. No rigid transform touches that; it is the ~89 %
precision / ~51 % recall map quality of §6b, and it is the thing that blocks
navigation, not the placement.

---

## 7c. `loop_closure:=false` -- a pure odometry chain (2026-10-26)

Added to `rtab_sim.launch.py` after the realistic-map A/B showed the front-end's
own closures were what corrupted an otherwise ~0.1 m map (ORB `loopClosing: 0`
took `_map` 2.01 -> 0.07 m, `_mapv2` 8.82 -> 0.10 m). The matching switch on the
back-end side lets a map be built with NO closures anywhere, so the grid is a
direct picture of odometry drift and the stitched map has something to be
compared against.

Two parameters, chosen so the database stays localisable:

| parameter | value | effect |
|---|---|---|
| `Rtabmap/LoopThr` | 1.0 | Bayes hypothesis is a probability; never crossed, so no appearance closure is ever accepted |
| `RGBD/ProximityBySpace` | false | no proximity links between nodes close in space |

`Kp/MaxFeatures` is deliberately left alone. Setting it to -1 would also stop
detection, and more cheaply, but it stops extraction too: the database would hold
no visual words and `mode:=localize` against it could never relocalise. With
`LoopThr 1.0` the words are still stored and only the acceptance is suppressed.
It is noted as a commented alternative in `_loop_closure_params()`.

Verified by resolving the launch description both ways: with the default the
rtabmap node's parameter set is byte-identical to before; with `false` exactly
the two parameters above are appended. Not yet run on a bag.

---

## 7d. Real rig: `camera_model:=fisheye` (2026-10-28)

§8's "the real rig cannot use this path" is retired. Both blockers moved:
the extrinsic is now a frozen stereo calibration (28 Oct, `camera_parameters_271026/281026*.dat`,
89.0 mm / 0.86 deg, written into `vins_fusion_ros2/config/wil/stereo_imu.yaml` in
inverse form), and the fisheye problem is solved by rectifying in the shim.

**What the real bag carries** (`dataset/dataset_realv5_000`, 134.6 s, recorded
28 Oct 15:31, i.e. AFTER the remount so the live extrinsic applies): stereo JPEG
`/cam{0,1}/image_raw/compressed` at 30 Hz, 1920x1080 mono, pair stamps within 4 ms;
`/clock` recorded (so NO `--clock`, same as sim); IMU on `/hwt101ct_yaw_publisher`;
no `camera_info`, no TF, no ground-truth topic (GT is the tracker file under
`dataset/gt`). `script/check_stereo_side.py`: 12/12 pairs negative, **cam1 is LEFT**.

**The shim's fisheye path** (`rtab_camera_shim.py`, `build_fisheye_rectification()`):
reads `cam{0,1}_fisheye.yaml` (KANNALA_BRANDT, VINS k2..k5 = OpenCV fisheye k1..k4;
cam1 reuses cam0's intrinsics because only one mono calibration exists) and
`stereo_imu.yaml`, takes R1/R2 from `cv2.fisheye.stereoRectify`, builds the
projection matrices BY HAND from `hfov_deg`, remaps both frames each tick, and
publishes `camera_info` + static TF to `cam{L}_rect_optical` / `cam{R}_rect_optical`
with `body_T_rect = body_T_cam * R^T`.

Three things it checks at startup, all of which failed silently at least once
while being written:

1. **`left_cam` against the extrinsic**: the other camera must sit at +x in the
   left optical frame. `left_cam:=0` exits with "cam1 sits at x=-0.0890 m".
2. **The rectified pair is canonical**: `rect_L_T_rect_R` must be `(B, 0, 0)` with
   identity rotation to 1e-6. This ties the maps and the TF together; measured
   `(0.0890, -0.0, -0.0)`.
3. **Comments are stripped before parsing** `stereo_imu.yaml`. The real config keeps
   five retired calibrations as commented blocks with the SAME `body_T_cam1:` key,
   and the regex parser (written for the sim file, which has none) returned the
   first one -- the pre-remount cam1 at **+0.0936 m**. The side check is what
   caught it. The sim values are unchanged by the fix.

`cv2.fisheye.stereoRectify`'s own P matrices are unusable for this lens: with
`balance=0` it returned **fx = 0.0** (a ~180 deg fisheye's corners map to infinity).
Only its rotations are used.

**Choosing `hfov_deg`.** Depth error per pixel of disparity is z^2 / (fx*B):

| hfov | fx (px) | fx*B (px*m) | cross-checked ORB matches on the epipolar line |
|---|---|---|---|
| 90 | 960 | 85.5 | 32-38 % |
| **110** | **672** | **59.8** | **49-58 %** |
| 130 | 448 | 39.9 | 66-74 % |

110 reproduces the sim's budget (59.9) and is the default. The match fraction
rises with FOV only because more of the fisheye survives; the inlier depth
estimate is the same at every focal length (2.3 / 3.5 / 4.6 m on the same three
frames), which is the check that the geometry is right rather than lucky.

**Offline epipolar check through the shim's own maps**, six frames across the
bag, 6689 inliers: vertical residual mean **-0.58 px**, |dy| p90 **1.73 px**,
disparity always positive. The residual bias is plausibly cam1 borrowing cam0's
intrinsics. Figures in the session scratchpad only (`realv5_rect_hfov*.png`).

**Live smoke test** (shim alone + `ros2 bag play dataset/dataset_realv5_000`, 25 s):
left `cam1_rect_optical` bgr8 1920x1080, right `cam0_rect_optical` mono8;
`camera_info` fx 672.2, cx 960, cy 540, right `P[0][3]` = -59.844; `tf_static`
`imu_link -> cam1_rect_optical` t = (-0.0890, -0.0012, -0.0023) and `-> cam0_rect_optical`
at the origin, both with the same quaternion. **Throughput: ~1 Hz effective at
`rate:=4.0`** -- two 1080p JPEG decodes + two remaps + 6 MB bgr8 per pair is
heavier than the sim's 720p path (3.2 Hz). Play the bag at `--rate 0.5` or lower
for mapping, or accept ~1 keyframe/s.

**Launch for the real rig** (ORB front-end, `wil_stereo_imu.launch.py`, publishes
`orbslam3_world -> imu_link`; ORB's `IMU.T_b_c1` equals VINS's `body_T_cam0`, so
`imu_link` IS the frame the shim's extrinsics are expressed in):

```bash
ros2 launch ./rtabmap_ros/rtab_sim.launch.py mode:=construct rviz:=true \
    camera_model:=fisheye left_cam:=1 hfov_deg:=110 \
    config_dir:=$HOME/wil_project/vins_fusion_ros2/config/wil \
    odom_frame:=orbslam3_world body_frame:=imu_link \
    ground_height:=<IMU height above floor, NEGATIVE, plus ~0.1> \
    output_path:=map/real/ database_path:=map/real/real.db \
    grid_range_max:=6.0 detection_rate:=4.0 shim_rate:=4.0 loop_closure:=false
# ros2 bag play dataset/dataset_realv5_000 --rate 0.5       (no --clock)
```

**Running it without starving ORB (2026-10-29).** ORB + shim + rtabmap did not fit
on this laptop together at 1080p (the sim had the same problem with VINS, §6d).
**Update 2026-11-04 (user report): live co-running ORB-SLAM3 and RTAB-Map now works.**
The exact conditions were not recorded here -- presumably the GPU front end
(patches 10/10b), `rect_scale:=0.5`, `detection_rate:=1.0` and a cool CPU; confirm
and write them down before quoting a live-mode map. The two-pass route below stays
valid and remains the deterministic choice for A/Bs. Two levers, use both:

1. **Two passes, replaying ORB's trajectory.** `script/gt_tf_publisher.py` reads the
   11-column `vio.csv` ORB writes (`trajectory_writer.cpp`: t_ns, xyz, qw qx qy qz,
   v), interpolates at `/clock`, and broadcasts it as TF -- so pass 1 is ORB alone
   at whatever rate it sustains, pass 2 is the bag + replayed TF + shim + rtabmap
   with ORB's whole CPU budget freed. Deterministic, A/B-able, and the realv5 ORB
   runs under `output/output_orb/real/realv5_*` already exist. Caveat: the
   interpolation bridges tracking gaps with a straight line, so pick a run with
   no losses.
2. **`rect_scale:=0.5`** (shim + launch arg, added today): rectified output at
   960x540, fx 336, fx*B 29.9. RTAB-Map's SGBM cost scales with pixels x disparity
   range, so this is the single biggest back-end saving; the 5 cm grid loses nothing
   visible. Pair with `detection_rate:=1.0` on the real rig.

**The ORB node's world on the real rig is the camera optical frame, not z-up
(2026-10-29).** First pass-2 attempt in RViz: floor plane ~90 deg off, no grid.
Analysed all six realv5 `vio.csv`: body y (physically up) -> world **-y** for the
whole run, trajectory plane normal = world y. The node publishes the ACTIVE map's
frame, which is the camera at map creation (x right, y down, z forward); the sim
only looked REP-103 because `align_first_pose` + an FLU IMU body made it so. There
is also a ~1 s transient after IMU init (91 deg flip at csv t=2.5 s and back at
2.8 s) and some runs flip again on a map reset (besteffort at 105.9 s). The ORB
Pangolin viewer is unaffected -- it draws its own gravity-aligned world -- which is
why "the viewer looks right" is not evidence about the TF.

Fixed in `script/gt_tf_publisher.py` (ground-truth replay unchanged by default):
`world_fix:=optical_to_flu` left-multiplies every pose by x'=z, y'=-x, z'=-y, and
`start_time:=3.5` drops the transient and sends NO TF until then (RTAB-Map waits
rather than anchoring its map in the wrong frame). Verified on
`realv5_imu_cbgroup_reliable_r05_newIMU_TO_CAM_TF`: after the fix body up -> +z
(|dev| < 0.04), plane normal (0.01, -0.03, -1), z extent 0.15 m over 130 s.
Forward is +x, right is -y. Use that run: 1366 rows, tracking state 2 throughout,
no reset after the transient.

**"No map at all" = a 0-node database = the shim died at startup (2026-10-29).**
`map/real/real.db` after the user's first full run: 0 nodes, 0 links, 0 Data rows.
Reproduced: the launch handed the shim `-p hfov_deg:=110` (the DeclareLaunchArgument
default is the string '110'), rclpy parsed it as INTEGER, and `declare_parameter`
of a DOUBLE with an integer override throws -- the shim's traceback scrolls past in
the launch output, rtabmap starts fine and simply never receives data. My earlier
live smoke test had typed `110.0` by hand, which is why it passed. Fix: the launch
now formats `hfov_deg`/`rect_scale` as floats AND the shim declares both with
`ParameterDescriptor(dynamic_typing=True)`. **Rule: a 0-node db means the input
chain is dead; read the shim's lines in the launch log before anything else.**

Rerun of the same pass-2 command on 55 s of `dataset_realv5_000` (bag on the flash
drive now): 29 nodes, 56 neighbour + 29 gravity links, every node with ground and
obstacle cells (median 22.6 kB obstacle / 1.5 kB ground); one TF-extrapolation
warning at the first frame only. Exported grid 263x240 at 5 cm: walls and the
corner the robot turned are visible, with the usual ray-tracing streaks from far
points. The static TF used a placeholder IMU height of 0.30 m.

**Second 0-node run, same symptom, different cause: frames with no odometry TF
yet (2026-10-29, later).** After the shim fix the user's run STILL had 0 nodes, and
so did three of my reproductions while one succeeded -- i.e. timing-dependent.
Mechanism, from the warning timeline: `start_time:=3.5` meant the replay sent NO
TF for the bag's first ~5.5 s, but the shim published pairs from the first frame.
RTAB-Map blocks `wait_for_transform` (5 s, wall) on EACH such frame inside its
subscription callback, serially; the ~11 pre-TF pairs cost ~55 s of blocking while
`topic_queue_size` 200 buffers everything behind them; by the time those are
drained their stamps are 20+ s behind sim time, outside the 10 s TF buffer ->
"Lookup would require extrapolation into the past" -> dropped -> and so on. The
console alternates "Did not receive data since 5 seconds" with one throttled
getTransform warning every 10 s. Neither `rviz:=true` nor `output_path` was the
cause (A/B'd both alone: 0 nodes each); they only shifted the timing.

Fix in `script/gt_tf_publisher.py`: `hold_before_start` (default true) broadcasts
the first KEPT pose from the moment /clock runs, so there is never a frame without
odometry. Valid because the robot is stationary through ORB's init transient
(csv positions constant until ~5 s). `hold_before_start:=false` restores the gap.
Verified with the user's exact arguments (rviz + output_path): 17 nodes in 45 s
of bag, all with grid cells, `/mapPath` written (13 rows).

**Rule for the next time the map is empty:** open the database and count nodes.
0 nodes = the input chain, in this order: shim traceback at startup; shim lag line
("N pairs; lag +X s"); rtabmap "Did not receive data" / getTransform warnings.
Nodes present but no grid = a Grid/* or base-frame problem instead.

**Process-cleanup trap that contaminated the later A/Bs.** `kill <pid of 'ros2 run
...'>` kills the Python wrapper and leaves the real `static_transform_publisher`
binary alive; `kill -9` on a `ros2 launch` PID orphans its children (shim, writer).
By the fourth reproduction two orphan shims were publishing `/stereo/*` alongside
the live one (rtabmap logged "Messages ... arrived out of order"; shim 4570 ms/pair,
RTAB-Map update 4.0-4.9 s vs 0.16-0.27 s clean) and the CPU sat at 92 °C /
0.8-1.3 GHz. Start test processes with `setsid` and kill the process group
(`kill -INT -- -<pgid>`), and check `ps -eo pid,ppid,lstart,args | grep shim`
before trusting any timing. The user's own `republish compressed raw` x2 and
`image_encoding_fixer` x2 from 16:37 were also still alive, decoding both 1080p
streams at 30 Hz during every bag replay.

**realv6 (2026-11-02): the replay settings are RUN-DEPENDENT, so they are now
measured from the csv.** With `world_fix:=optical_to_flu start_time:=3.5` the
realv6 map built (grid + cloud) but pose/TF were wrong. The realv6 ORB run
(`loopclosing_rate1`, LC on but 418/418 place-recognition no_candidate, so no
closure jumps) has its transient until **9.5 s** (jumps at 2.5/2.6/5.3/5.4/9.5 s)
and afterwards is already **gravity-aligned z-up** (body up -> +z, plane normal
-z); the fixed optical_to_flu rotation then tipped a correct world onto its side,
and 3.5 s was inside the transient. `gt_tf_publisher.py` gained
`world_fix:=auto` (rotate so the mean body-up axis -- `body_up`, default (0,1,0)
for this IMU frame -- becomes +z; yaw kept; also levels a constant mount pitch)
and `start_after_transient:=true` (0.5 s after the last >45 deg jump in the first
30 s). Offline: realv5 newTF -> 90.8 deg rotation, start 3.3 s (= the hand-tuned
values); realv6 -> 0.5 deg, start 10.0 s. Spread of the body-up axis over each run
is 1.7 / 2.8 deg p95, i.e. the floor is flat enough for the levelling to be real.
Use these two options for every replay from now on; the fixed ones stay for sim GT.

**realv6 dense map unidentifiable = the rig moved; the 28 Oct extrinsic no longer
rectifies it (2026-11-02).** `map/real/real.db` (realv5, triangle route) shows cars;
`map/real_311026_rate05_lc/` (realv6, 8-shape) is a radial smear. Ruled out first:
not the stored calibration (both dbs carry the same 960x540 pair and encoding), not
the 44 accepted closures (median correction 0.14 m, max 0.67 m; the odometry-only
re-export of the same db is the identical smear), not scene distance (realv6 points
are nearer the path, p50 1.5 m vs 2.1 m), not image quality (brightness, sharpness
equal). The discriminator was epipolar alignment of the STORED rectified pairs:
realv5 79-87 % of cross-checked ORB matches within 2 px of the line, realv6
**7-22 %** with a -1.2 px median residual. SGBM at blockSize 15 cannot match
across that, so every pixel's depth is noise along its ray = the smear. On the raw
bags with every chequerboard calibration on disk (10-21, 10-26, 10-27, 10-27_01,
10-28): best on realv6 is 15 % (10-27), 10-28 gives 1 % (dy -6 px) where it gives
46 % on realv5. **The baseline direction moved 6.3 deg between 28 Oct and 31 Oct** (realv6 was recorded 31 Oct 19:00; its node stamps say so, the `311026` dir names are the recording date)
(rotation only 0.45 deg) -- a mount change, same class of event as 10-19 and 10-28.
ORB was NOT visibly hurt (map inliers median 883 vs 735 on realv5): its
KannalaBrandt stereo path triangulates with a looser check, so a good ORB
trajectory is no evidence that the extrinsic still rectifies.

`script/stereo_selfcal.py` (new): essential matrix over fisheye-undistorted ORB
matches from ~8 frames gives R and the baseline DIRECTION; the length is copied
from a reference .dat (unobservable from images). Validated on realv5: 0.2 deg
from the 28 Oct rotation, on-line 46 -> 77 %. On realv6: on-line 1 -> 43 % on raw
frames, and a map rebuilt with it (`vins_fusion_ros2/config/wil_311026_selfcal/stereo_imu.yaml`,
inverse-form derivation, baseline 89.0 mm) stores pairs at **84 % on-line**, SGBM
valid 62 %, 36 nodes in 45 s, update 0.31 s; the top view shows wall lines and
object outlines again, with residual ray streaks (self-cal still leaves a ~1 px
vertical bias -- a chequerboard calibration should do better). The candidate .dat is
`camera_parameters_311026_selfcal/311026camera1_rot_trans_selfcal.dat` -- a stopgap until a chequerboard
calibration of the current mount; NOT written into config/wil (user manages the
calibration history there).

**Rule:** before trusting any real-rig map, run
`python3 script/stereo_selfcal.py <bag> --ref <live .dat>`; under ~30 % on-line the
dense map will be noise whatever RTAB-Map does.

**Measured stereo depth noise vs range on the real rig (2026-11-02).** Floor-plane
fit on SGBM depth of stored 960x540 pairs (rect_scale 0.5, fx*B = 29.9), median
|residual| per range bin, several nodes each:

| range | realv5 (28 Oct, good) | realv6 self-cal | realv6 misrectified | 1 px theory |
|---|---|---|---|---|
| 0.5-1 m | 0.9 cm | 4.1 cm | 3.3 cm | 2 cm |
| 1-1.5 m | 2.6 cm | 6.3 cm | 5.6 cm | 5 cm |
| 1.5-2 m | 6.5 cm | 4.9 cm | 12.6 cm | 10 cm |
| 2-3 m | 6.6 cm | 7.0 cm | 29.5 cm | 21 cm |
| 3-4 m | 13.8 cm | 13.4 cm | 32.6 cm | 41 cm |
| 4-6 m | 60.8 cm | 24.6 cm | 72.2 cm | 84 cm |

So at half scale the depth is grid-cell accurate (<= 5 cm) only to ~1.5 m and
within the 25 cm scoring tolerance to ~3-4 m; past 4 m it is worse than half a
metre even with a good calibration. The user's "about 1 m" instinct is right for
the 5 cm cell. `grid_range_max:=6.0` is too generous for the real rig at
rect_scale 0.5 -- use ~3 m, or rect_scale 1.0 to stretch each bound by ~1.4x. The
self-cal's ~1 px vertical bias shows as the worse near-range numbers (4 vs 1 cm);
the misrectified column is 2-4x worse at 2-4 m, which is the smear.

**`extend` mode first exercised, on a real database (2026-11-04).** `mode:=extend` on
`map/realv7_001_lc/realv7_001_lc.db` (session 0: 80 nodes, 123 s) with the live
ORB front end added session 1 (70 nodes, 85 s, a different window of the same
recording, 254 s earlier) and stitched it: 96 cross-session links (58 global
closures + 38 local-space), 150 nodes total. Correctness of the closures is
unscored (no real-site ground truth); the sim rule still applies -- a closure
correction above ~1 m is suspect. Two traps found on the way: duplicated launch
arguments (last wins) and a directory `database_path` resolving to a NEW
`rtabmap.db` -- the launch now picks the single .db in a directory and refuses
extend/localize on a missing file. `delete_db` is still refused in these modes.

**`imu_link` is NOT a usable base frame.** ORB's `IMU.T_b_c1` = diag(1,-1,-1) makes
`imu_link` x-right, y-UP, z-BACKWARD, and `Grid/MaxGroundHeight` is measured along
base_frame z. Publish a FLU `base_link` under it and pass `base_frame:=base_link`:

    ros2 run tf2_ros static_transform_publisher --x 0 --y -<IMU height m> --z 0 \
        --roll 0 --pitch 1.5708 --yaw 1.5708 --frame-id imu_link --child-frame-id base_link

(R = Rz(90)Ry(90): base forward = -z_imu, left = -x_imu, up = +y_imu, verified
numerically.) Then `ground_height` keeps its sim meaning (~0.1 above the floor).

**2026-11-04: IMU height measured = 0.11 m** (user's static TF `--y -0.11`), and the
`imu_link -> base_link` transform is now published BY THE LAUNCH: `frontend:=orb|vins`
sets odom/body frames (ORB real: orbslam3_world -> imu_link; ORB sim: -> base_footprint;
VINS: world -> body), `publish_base_tf:=auto` adds the static publisher only for
`camera_model:=fisheye`, `imu_height:=0.11` default, `base_frame` defaults to base_link
then. Terminal 2 of the real-rig procedure is gone. Live co-running works since the
cooling fan is on (thermal was the limit, not the pipeline).

Not yet done (SUPERSEDED above for the height): `ground_height` for the real rig is unknown (the IMU's height above
the floor has not been measured; the sim's 0.29 is in `base_footprint` terms), the
real ORB config has `loopClosing: 1` (set 0 for the closure-free map, back to 1
to load the Atlas), and no RTAB-Map database has been built from real imagery
yet. The pinhole path was NOT re-run live after this change (no sim bag on the
local disk); it was verified by parameter resolution (byte-identical shim
arguments plus the three new ones at their defaults) and by re-parsing the sim
extrinsics (unchanged).

---

## 8. Open items

**Where the two scripts live -- RESOLVED 2026-11-04.** `rtab_sim.launch.py`,
`rtab_camera_shim.py`, `rtab_trajectory_writer.py` and `rtab_map.rviz` moved to the
new `vslam_navigation` package (`launch/rtabmap.launch.py`, `scripts/`, `rviz/`),
built with `colcon build --symlink-install --packages-select vslam_navigation`.
`rtabmap_ros/rtab_sim.launch.py` is now a forwarding stub (CLI arguments are
inherited by the included description), so every command in this record still
runs. `vslam_navigation/launch/mapping_real.launch.py` is the one-command real-rig
mapping bring-up: ORB-SLAM3 (wil_stereo_imu) + rtabmap.launch.py with
frontend:=orb camera_model:=fisheye, `map_name:=` and `calibration:=` (wil |
wil_311026_selfcal, both read from the SOURCE config trees), RTAB-Map started
`rtabmap_delay` 5 s after ORB so no pre-odometry frames block wait_for_transform.
`localize_real.launch.py` (same shape, mode:=localize, read_only default true, loop
closure forced on, refuses a missing prior) was added the same afternoon.
**Trap found on the first real runs (4 Nov 14:14-14:21): launch-argument name
collision.** Launch configurations are inherited by included launches, so a parent
argument named like a child's (`imu_accel_scale`, declared with default '' and only
forwarded when non-empty) handed the ORB launch an EMPTY string -> orbslam3_node died
in < 1 s with "parameter 'imu_accel_scale' has invalid type ... string" on all three
runs; the user had started ORB by hand without noticing. Renamed to
`orb_imu_accel_scale`; probe run shows the node loading settings with 0.9923. Rule:
never give a bring-up launch an argument with the same name as an included launch's
argument unless it is always forwarded explicitly.
**Second leak of the same kind (14:37 run, realv7_000 replayed correctly):** an
include's `launch_arguments` are SET in the shared context and stay there, so the
rtabmap include in localize_real inherited the ORB include's `output_path`; the
RTAB-Map trajectory writer then wrote the PRIOR map's graph (98 rows, realv7_001
stamps) over ORB's vio.csv (1759 poses; `final_frame_trajectory.txt` still has
them). Fixed by pinning `output_path: ''` in the rtabmap include. Rule: every
argument an included launch declares must be passed explicitly when two includes
share a context. `scripts/localization_recorder.py` added (started by
localize_real): /localization_pose with an inside-extent flag from the prior db's
node bounds (+margin), the map->odom correction at 2 Hz with its step, and a
summary line -> `map/<map>/localize_<run>.csv`. Probe without a bag: extent of
realv7_001_prior x -5.3..0.0, y -9.1..0.8 m; summary "poses 0, first correction
never" as expected. The 14:37 run itself left no relocalisation evidence.
**First relocalisation on a real prior map -- 2026-11-04 14:46, realv7_000 on
`realv7_001_prior` (108 nodes, built from realv7_001; both bags 31 Oct, same mount,
calibration `wil`).** `map/realv7_001_prior/localize_realv7_000_on_prior.csv`:
74 localisation poses at 1 Hz from bag +4 s to +91 s with no gap > 3 s, **100 %
inside the prior's extent**, covariance trace median 0.021. The map->odom correction
was re-seated twice in the first seconds -- +4.5 s (-3.38, 0.99 m, 87.9 deg) then
+5.5 s (-0.36, -1.88 m, -8.2 deg), a 4.2 m jump of the robot's map pose -- and then
held for 85 s with step median 0.000 m, p95 0.026 m, max 0.000 m after +5.5 s. The
two early steps coincide with ORB's IMU initialisation (imu_initialization_time
2.5 s) and its post-init frame re-alignment, not with a later aliasing event; ORB
had 0 resets and 3 losses totalling 0.1 s. The robot's map trajectory (x -5.6..0.0,
y -6.9..1.0) lies inside the prior's node extent (x -5.3..0.0, y -9.1..0.8). No
ground truth, so "correct" rests on consistency: an 85 s run with centimetre
corrections and every pose inside the map. Rule for judging such runs: ignore
corrections during the front end's init transient (first ~6 s), then apply the
> 1 m step rule. Run was fully from `localize_real.launch.py` (no separate ORB
terminal, 6 processes clean).
**Nav2 layer built and tested offline (2026-11-04 afternoon; robot broken, so no
drive test).** `vslam_navigation` now has `params/nav2_real.yaml` (from the sim's
nav2_ackermann.yaml: odom frame orbslam3_world, rolling local costmap in that frame,
/map from RTAB-Map use_saved_map, no live obstacle layer, speed cap 0.6 m/s,
use_sim_time rewritten at launch), `behavior_trees/` (sim BT copied),
`scripts/pose_gate.py` (init transient >= 8 s, correction step < 1 m for 6 samples,
robot inside the prior's extent + 2 m -> /localization_ok, relays
/localization_pose_gated, STARTUP/PAUSE/RESUME of the Nav2 lifecycle manager),
`scripts/ackermann_bridge.py` (/cmd_vel -> /cmd_velocity + /cmd_steering, bicycle
model, zero on stale cmd or gate false, dry_run default TRUE; wheel_base 0.42,
steering limit 0.6109, topic units all UNVERIFIED on the real chassis),
`launch/nav2_real.launch.py` (also the offline harness: map_yaml:= starts a
map_server), `launch/navigate_real.launch.py` (localize_real + nav2_real + RViz
`rviz/navigation.rviz`). Offline harness run on `realv7_001_prior` exported to
pgm, static TFs for map->orbslam3_world (identity) and ->base_link at node 15, goal
at node 75 (5.05 m away): gate went OK after the 8 s window, STARTUP -> "Managed
nodes are active", goal accepted, 62 plans of 69 poses, 1369 /cmd_vel at v median
0.46 m/s (regulated below the 0.6 cap), max curvature |w/v| 1.08 1/m under the
1.67 rear-axle bound; bridge dry-run showed steering held at 0.425 rad while the
smoother ramped velocity (scale_velocities working); "Failed to make progress" and
"spin failed" are the expected consequences of a robot that cannot move / cannot
rotate in place. Untested: the whole chain live (navigate_real on a bag replay
with ORB + RTAB-Map localize), and everything on the robot.
`navigate_real.launch.py` run on realv7_000 against `realv7_001_prior` (15:31-15:35,
15 processes, all clean, bridge dry-run): localisation again 73 poses, 100 % inside
the extent, one early re-seat (3.83 m) then stable -- the third consistent
relocalisation of this pair. Whether the gate opened and a goal was planned was not
captured (nav2/gate output goes to the screen only), so the gate now also writes
`map/<map>/gate_<run>.csv` (transitions + lifecycle commands).
**Real chassis parameters (user, 6 Nov 2026) and the command interface measured
from dataset_realv7_001:** wheel_base 0.46 m, steering limit +-20 deg, footprint
+0.13/-0.52 m along x, +-0.21 m along y about base_link, target 1.5 m/s.
Derived: rear-axle curvature 0.791 1/m (R 1.264 m), base_link R 1.345 m ->
`minimum_turning_radius` 1.45, RPP `min_lookahead_dist` 2.70 (>= 2 x 1.345),
max yaw rate 1.2 rad/s; chosen accelerations 0.5 / -1.0 m/s^2 (3 s to 1.5 m/s,
1.1 m stop). `/cmd_steering` is DEGREES and POSITIVE = RIGHT: 10/10 windows of
+-30 deg commands gave the opposite-signed body turn rate (0.19-0.26 rad/s at
0.29 m/s, R ~1.4 m at the IMU, i.e. the +-30 commands saturate at ~18-20 deg) ->
bridge steering_scale 57.3, steering_sign -1. `/cmd_velocity` 1.25 held 112 s
gave 0.29 m/s median (p90 0.37) -> velocity_scale 4.3 PROVISIONAL (one operating
point; realv2's /feedback/velocity read 3.0 x tracker speed, so the unit is
unknown and the response may be nonlinear); bridge caps the published command at
`max_command_velocity` 2.5 units until two more points are measured. Offline
harness rerun with these values: gate -> Nav2 active, goal accepted, 59 plans of
130 poses (longer: the 1.45 m radius), cmd_vel 1.38 m/s, max curvature 0.68 < 0.79,
bridge dry-run steering -17.3 deg. Also fixed: an EMPTY `-p log_file:=` kills a
rclpy node at init ("Couldn't parse parameter override rule") -- pass optional
parameters only when set.
Next steps in that package: re-run navigate_real once so the gate log exists;
measure wheel_base, steering limit, footprint and the /cmd_* units on the robot;
Nav2 (from the sim's nav2_ackermann.yaml), RViz config. Paths quoted below this
line are historical.

**Where the two scripts live (original note).** They are currently inside this fork
(`rtabmap_ros/rtab_camera_shim.py`, `rtabmap_ros/rtab_sim.launch.py`). That is
almost certainly wrong: they are WiL project code, not upstream RTAB-Map code,
and will collide or be lost on any upstream merge. The workspace convention puts
helpers in `script/`. Unresolved.

**The real rig cannot use this path -- RETIRED 2026-10-28, see §7d.** The
original reasoning, kept for the record: `config/wil/cam0_fisheye.yaml` is a
`KANNALA_BRANDT` ~180° fisheye at 1920×1080. RTAB-Map's stereo pipeline assumes a
rectified pinhole pair and has no equidistant model — VINS can consume it, RTAB-Map
cannot. Rectifying to a ~90–100° virtual pinhole discards the periphery, which is
where a wide-FOV rig earns its loop closures. Also `config/wil/stereo_imu.yaml`
sets `estimate_extrinsic: 1`, so `body_T_cam0` is solved online and there is no
fixed extrinsic to build a static TF from until a calibration is frozen.

**ORB-SLAM3's trajectory is not the optimised one.** `orbslam3_node.cpp:305`
writes `vio.csv` per frame inside `onTrackResult` and never revises it, and
`SaveTrajectoryTUM` (`System.h:147`) is never called — `slam_wrapper.cpp:85` calls
only `Shutdown()`. When ORB-SLAM3 closes a loop it corrects the *map*, so poses
after the closure are good but rows already flushed keep their pre-loop drift,
leaving a discontinuity mid-file. Since ATE alignment is a rigid Umeyama fit over
the whole trajectory, that jump also pulls the alignment and inflates residuals on
both segments. **Add `SaveTrajectoryTUM` and re-measure before drawing any
conclusion about what RTAB-Map adds to ORB-SLAM3** — it is a one-line change and
it establishes the honest baseline.

**Two comparisons, not one.** VINS has no loop closure, so VINS-vs-live-stream-
ORB-SLAM3 is a consistent "online odometry" comparison, while
VINS-vs-`SaveTrajectoryTUM` is odometry-vs-full-SLAM. Report both columns rather
than picking.

**Odometry covariance is discarded** (§6). If graph quality proves limiting, the
fix is to feed the topic *and* raise `topic_queue_size` far enough, or throttle
the odometry — not to revert to the topic naively.

**Disk.** The workspace sat at 98–99 % full during this work (849 MB free at one
point); the build needs ~450 MB and an RTAB-Map database ran to 15 MB for 40 s.
Currently 8.9 GB free.

## Live operation from the laptop: abandoned (6 Nov 2026)

First attempt at `localize_real.launch.py` against the live robot instead of a bag.
The image topics appeared unreadable. Two separate causes, one of them mine.

**Measurement artefact.** `timeout N ros2 topic hz /t | tail` prints nothing even for a
200 Hz topic: Python block-buffers stdout into a pipe and SIGTERM kills it before the
flush. Every rate probe looked like silence while `ros2 topic echo --once` on the same
topics returned immediately. Correct form:

    export PYTHONUNBUFFERED=1
    timeout -s INT 20 ros2 topic hz /t --window 50 > out.txt 2>&1

**Real cause.** Compressed frames are 0.52 MB each at 1920x1080 and the publishers are
RELIABLE.

| subscribers | rate | throughput |
|---|---|---|
| cam0 alone | 25.9 Hz | 13.26 MB/s |
| cam0 + cam1 | 0 / 5-7 Hz | 0.19 / 0.22 MB/s |

Both streams together deliver ~0.41 MB/s, a 32x collapse rather than a split. Each
sample fragments into hundreds of datagrams; one lost fragment retransmits the whole
sample, so beyond capacity nothing completes. No stereo pair ever arrives, so the
`message_filters` synchroniser in both the ORB node and `rtab_camera_shim.py` never
fires. Buffer and QoS tuning on the subscriber side cannot change this arithmetic.

Decision: clone the repo onto the Jetson and run the stack there. Bag replay on the
laptop is unaffected. All five launch/script files resolve `~/wil_project`, so a clone
at the same path needs no edits.

**Still open for live running, wherever it runs.** `rtabmap.launch.py` hard-codes
`use_sim_time: true` in four places while `wil_stereo_imu.launch.py` sets it false in
two. On bags that was correct. Live, the robot publishes `/clock` at 62-75 Hz from
`robot_can_interface`, measured 153 s behind the laptop wall clock, so the laptop
mixed two timelines and every TF lookup would have failed. On the Jetson both clocks
have the same origin and the mismatch is harmless, but `/clock` stalling would freeze
the RTAB-Map side while ORB keeps running. A single `use_sim_time` argument threaded
from `localize_real.launch.py` into both includes is the fix; not yet written.

## mapping_sim.launch.py: the simulation wrapper (6 Nov 2026)

`mapping_real.launch.py` starts ORB-SLAM3 itself, so the rig needs two terminals
(launch + bag). Simulation needed three until now. `mapping_sim.launch.py` closes
the gap:

    ros2 launch vslam_navigation mapping_sim.launch.py \
        map_name:=dataset_allsensor_realistic_01_map loop_closure:=false
    ros2 bag play "/media/.../dataset_allsensor_realistic_01_map" --rate 0.3

**Why mapping_real cannot be reused for simulation.** Tried first, and it produces a
zero-node database rather than an error:

  * `config_file` is NOT one of its seventeen arguments. It exists inside the file
    only as the value derived from `calibration` and handed to the ORB launch, and
    ros2 launch drops undeclared arguments silently (the `accel_scale` lesson). A
    command-line `config_file:=.../wil_sim/stereo_imu.yaml` is therefore ignored and
    ORB loads the real fisheye calibration.
  * it hard-codes `camera_model: 'fisheye'` with no override, so the shim would run
    Kannala-Brandt rectification on frames that are already ideal pinhole, using
    rig intrinsics.
  * fisheye + frontend orb selects `imu_link` as the body frame and publishes
    `imu_link -> base_link`; in simulation ORB broadcasts to `base_footprint`.
  * `wil_stereo_imu.launch.py` sets use_sim_time false (the real bags carry no
    /clock); the simulation bags carry 315 203 clock messages and the simulation ORB
    launch sets it true.

Confirmed empirically: `map/dataset_allsensor_realistic_01_map.db` written by the
mapping_real attempt at 18:25 held 0 nodes, 0 links, 0 words (106 496 bytes, the
empty-database size).

**What the wrapper pins.** camera_model pinhole; front end
`orbslam3_ros2 wil_sim_stereo_imu.launch.py` (defaults to config/wil_sim, sim time
on); `image_transport:=compressed` because the simulation bags are compressed-only;
no imu_height, since publish_base_tf 'auto' acts only on the fisheye path;
`orb_output_path` renamed away from the child's `output_path` for the usual
inheritance reason. It refuses a fisheye calibration directory (checks for
cam0_pinhole.yaml) and refuses `mode:=extend` with no existing database.

Verified 6 Nov on dataset_allsensor_realistic_01_map: 19 nodes in 60 s of replay,
closure-free, occupancy cells on all 19, extent 11.0 x 27.5 m over 36.2 m of path.

**Shim shutdown fix found by that run.** A SIGINT invalidates the rclpy context while
message_filters is mid-callback, so `pub_left.publish()` raises RCLError, which is
neither KeyboardInterrupt nor ExternalShutdownException and escapes `rclpy.spin()`.
Every clean Ctrl-C was reported as `process has died [... exit code 1 ...]` with a
twenty-line traceback -- pre-existing, and it affects mapping_real identically.
`rtab_camera_shim.py` now returns early from `on_pair` when `rclpy.ok()` is false and
swallows the lost-race exception in `main` only once shutdown has begun, so a real
mid-run fault stays loud. All four processes now report "finished cleanly".
RCLError is not importable from `rclpy.exceptions` in Humble, hence the context guard
rather than an except clause on the private pybind symbol.

**Replay rate.** At `--rate 0.5` ORB reported "dropped 3 of 525 stereo frames";
its own warning says any drop makes two runs incomparable. Use 0.3 or lower for a
map that will be scored.

## Simulation parity: localize_sim / nav2_sim / navigate_sim (6 Nov 2026)

The rig had mapping, localize, nav2 and navigate; simulation had only mapping (and
that only since earlier today). The gap is now closed, in `vslam_navigation`:

| rig | simulation |
|---|---|
| mapping_real.launch.py | mapping_sim.launch.py |
| localize_real.launch.py | localize_sim.launch.py |
| nav2_real.launch.py | nav2_sim.launch.py |
| navigate_real.launch.py | navigate_sim.launch.py |
| params/nav2_real.yaml | params/nav2_sim.yaml |

    ros2 launch vslam_navigation navigate_sim.launch.py map_name:=<prior> \
        world:=.../small_warehouse_static_realistic_nofloortexture_01.world

**What already existed and is REUSED, not rewritten.**
`aws_robomaker_small_warehouse_world localization_rtabmap.launch.py` already builds
the whole SLAM frame tree (map -> odom -> vio_body -> base_footprint -> base_link,
one publisher per edge, front ends told to publish odom -> vio_body directly) and
already pins `output_path: ''` and `rviz: false` for its caller. `localize_sim`
includes it and adds only what it lacks: a `map_name` interface so the database and
the CSV land together under map/<name>/, and `localization_recorder.py`.

**Why the warehouse package's nav2.launch.py is not the answer.** It includes
`localization_tf.launch.py`, so it navigates on GROUND TRUTH, whose map -> odom is
an exact static identity. That is the right tool for testing Nav2 itself and cannot
answer whether the robot can navigate on a map it built and relocalised into. The
two are mutually exclusive, not composable: both publish map -> odom.

**What nav2_sim.yaml changes from nav2_ackermann.yaml, and nothing else.** The
chassis half (wheel_base 0.42, 35 deg, minimum_turning_radius 0.75, lookahead
1.30/2.00, footprint, smoother limits) is untouched -- the chassis did not change.
Changed, all because localisation changed:
  * local costmap is ROLLING 10x10 m. The source file states its non-rolling
    costmap in the odom frame is safe "only because map->odom is a permanent, exact
    identity". Under SLAM localisation that premise is false.
  * odom_topic /orbslam3/odometry, not /odom: nothing publishes /odom here, because
    ground_truth_localization is what we replaced.
  * autostart false, so pose_gate.py starts the stack when localisation is sound.
  * allow_unknown stays false but now BITES: a stereo-built prior has unknown space
    where the baked map had none, so some goals are refused for that reason.
  * cache_obstacle_heuristic false: the static layer is republished when RTAB-Map
    updates the grid.
  * use_collision_detection FALSE. The source file's own sweep records ON at
    lookahead 1.30 giving "paralysis, 155 vetoes, 0.10 m/s", and a stereo prior is
    less trustworthy than a baked one. Note nav2_ackermann.yaml's comment says OFF
    while its value says true; the two disagree there.

**No ackermann_bridge in simulation, and that is structural.** gz AckermannSteering
takes a Twist on /model/<robot>/cmd_vel and small_warehouse's cmd_vel_bridge already
forwards /cmd_vel. The chain stops at /cmd_vel. The rig needs /cmd_velocity and
/cmd_steering in its own units instead. pose_gate.py and localization_recorder.py
needed NO changes: both already parameterise map/odom/base frames, so simulation
just passes odom_frame:=odom.

**Verified, not merely written.** Offline harness (static TF stand-ins, map_server
on maps/baked_dynamic_realistic_01, no Gazebo, no estimator), start (-6.4, 10.95)
to goal (7.4, -9.85), a pair chosen because the straight line between them crosses
84 blocked cells:

    all 7 lifecycle nodes      active
    plans                      51, first 237 poses / 28.53 m in frame map
    cmd_vel                    1029 msgs, linear.x max 1.500 m/s, |w| max 0.565
    curvature max              0.377 1/m against the rear-axle bound 1.6673 -> feasible

A bare `nav2_sim.launch.py` with no TF and no map stalls at "Configuring
controller_server" with `Invalid frame ID "odom"` repeating from local_costmap: that
is the costmap waiting for odom -> base_link, not a parameter fault. Every plugin
configures, Smac included.

**World must match the prior.** `world` defaults EMPTY so small_warehouse's own
default is used and cannot drift from it; the realistic worlds are
small_warehouse_static_realistic_nofloortexture.world (layout U) and
..._nofloortexture_01.world (layout I), and the realistic_01 bags are layout I.
An earlier draft defaulted it to a no_roof_small_warehouse.sdf path that does not
exist (the file is .world).

## Three bugs from the first real navigate_sim run (6 Nov 2026)

The user ran navigate_sim twice: once alongside a separate small_warehouse terminal,
then once on its own. Both aborted. Three distinct faults, two of them mine.

**1. Reading the log. Only the FIRST error matters.** Both logs ended in a wall of

    terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
      what(): could not create service: rcl node's context is invalid
    process has died [... exit code -6 ...]

from robot_state_publisher, joint_state_publisher and the vio_body static TF, plus
`ign gazebo ... failed to terminate 5 seconds after SIGINT`. None of those are
faults. navigate_sim starts the localisation layer on a TimerAction at +12 s, so
when the launch aborts at t<12 s the shutdown lands exactly while those nodes are
constructing, and they abort with an invalid context. The real error is at the TOP.

**2. frontend spelling, localize_sim's bug.** `vslam_navigation` spells ORB-SLAM3
`orb` everywhere; `localization_rtabmap.launch.py` validates `vins|orbslam3`.
localize_sim passed `orb` through, giving

    frontend must be vins|orbslam3, got 'orb'

Fixed by translating at the boundary (ALIAS dict), not by changing the package's
own spelling.

**3. frontend LEAK, a latent bug in localization_rtabmap.launch.py.** With that
fixed the error inverted:

    frontend must be orb|vins (or empty), got 'orbslam3'

because launch configurations are inherited by included launches and that file
never pinned `frontend` for its rtab include, although it sets odom_frame,
body_frame and base_frame explicitly -- and rtabmap.launch.py's contract is
"explicit odom_frame/body_frame win when frontend is empty". Its default `vins`
hid this, being a value the child also accepts, so only the ORB path ever broke.
Fixed by adding `'frontend': ''` to that dict.

## Closure-free priors could never be relocalised against (fixed 6 Nov 2026)

The run that exposed this logged, in localize mode:

    Update RTAB-Map parameter "Rtabmap/LoopThr"="1.0" from database
    Update RTAB-Map parameter "RGBD/ProximityBySpace"="false" from database

RTAB-Map restores its parameter set FROM THE DATABASE. A prior built with
`loop_closure:=false` therefore comes back with closures disabled even when the
launch is asked for them, and `_loop_closure_params(enabled=True)` returned `{}`,
setting nothing to stop it. In mode:=localize that is fatal and silent, because
relocalising against a prior IS accepting a loop closure against it: the node
starts, publishes /localization_pose, and never localises.

This is the explanation for the real-rig result scored earlier the same day:

| prior | closures in db | localisation cov_trace |
|---|---|---|
| map_prior_v2 | 0 | 3e+04 on all 76 poses (no-information value) |
| realv7_001_prior | 32 | median 0.022 |

map_prior_v2 was not a bad relocalisation, it was no relocalisation at all. Its
correction yaw drifting 0 -> 112 deg and the single 1.20 m step were the pose being
dragged by odometry, not a closure being accepted.

`_loop_closure_params(True)` now returns RTAB-Map's own defaults explicitly,
`Rtabmap/LoopThr 0.11` and `RGBD/ProximityBySpace true`, and rosparam wins over the
stored value. Verified: the log now says "Setting ... (rosparam)" for both instead
of "Update ... from database".

CONSEQUENCE FOR SCORING: every localisation run made against a closure-free prior
before this date is void, on both the rig and in simulation. Re-run them. The
diagnostic is cheap -- cov_trace 3e+04 in the recorder CSV means nothing localised.

## Real chassis: wheel base corrected to 0.60 m (user, 7 Nov 2026)

The rig's wheel base is 0.60 m, not the 0.46 m recorded on 6 Nov. What changed
and what did not:

**Unchanged.** Experiment 1's real-robot baseline. `script/real_wheel_baseline.py`
records the wheelbase but does not use it -- the rig publishes no steering
measurement, only a two-valued command echo, so no bicycle model is formed. The
0.22-0.46 m dead-reckoning results stand.

**Changed.** Everything derived from the Ackermann geometry in
`vslam_navigation`, recomputed at the 20 deg steering limit:

| quantity | L = 0.46 | L = 0.60 |
|---|---|---|
| rear-axle curvature tan(d)/L | 0.791 1/m | 0.607 1/m |
| R_rear | 1.264 m | 1.649 m |
| base_link radius sqrt(R_rear^2 + L^2) | 1.345 m | 1.754 m |
| minimum_turning_radius | 1.45 | 1.85 |
| lookahead min / dist / max | 2.70 / 2.70 / 3.50 | 3.55 / 3.55 / 4.50 |
| regulated_linear_scaling_min_radius | 1.60 | 2.00 |
| max yaw at 1.5 m/s | 1.19 rad/s | 0.91 rad/s |
| smoother max_velocity yaw | 1.20 | 0.91 |
| behaviour max_rotational_vel | 1.2 | 0.91 |
| turning circle | 2.7 m | 3.5 m |

Also updated: `ackermann_bridge.py` default (the wheelbase is LIVE there, in
`steer = atan(L*w/v)`), the `wheel_base` launch defaults in `nav2_real` and
`navigate_real`, and the report (chapter 3 twice, chapter 4 via the Experiment 1
note plus `build_chapter4.py`, chapter 5).

A 3.5 m turning circle in a 10 x 7 m zone is half the short dimension. Expect
Smac to reject more goals than the earlier 2.7 m figure implied; that is correct
behaviour, not a regression.

**UNRESOLVED: the steering limit, not the wheelbase.** The realv7_001 cross-check
(ORB trajectory against /cmd_steering +-30) measured a turn rate of 0.19-0.26
rad/s at 0.29 m/s, i.e. R ~1.4 m at the IMU. At L = 0.46 that implied an 18 deg
wheel angle, consistent with the stated +-20 deg; at L = 0.60 the same
measurement implies 23 deg, ABOVE the limit. Either the limit is nearer 23 deg,
or the radius was optimistic (R = 1.6 m gives 20.6 deg). The configs keep 20 deg
deliberately: it plans a wider circle than the car may need, and planning wider
than the car can turn is the safe error. Re-measure on a figure-of-eight at full
lock before raising it.

## Chapter 3 now describes this package (7 Nov 2026)

`docs/report/final_report/chapters/chapter_3_methodology.tex` gained
section 3.3's last subsection, `sec:nav-integration`, written in-chapter (not a
fragment): pose by transform rather than by topic, the stereo shim's two domain
cases, the three memory modes with the closure-free-prior reasoning and the
LoopThr-from-database defect, the pose gate's three conditions, the command
interface with the bicycle inversion, a task/entry-point table
(`tab:nav-integration`) and a status paragraph. The framing is that the package
is the INSTRUMENT answering the second half of Experiment 4's question ("is the
map usable by a route planner"), while path planning stays outside the evaluated
scope -- no navigation result is reported. A `Navigation integration / added` row
was added to `tab:deviations`, objective 5's status now notes the bring-up
exists, and the coordinate-frame subsection gained the label
`sec:frames-contract` so it can be cross-referenced. Report compiles, 133 pages,
no undefined references.
