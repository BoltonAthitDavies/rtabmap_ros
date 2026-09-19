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

## 8. Open items

**Where the two scripts live.** They are currently inside this fork
(`rtabmap_ros/rtab_camera_shim.py`, `rtabmap_ros/rtab_sim.launch.py`). That is
almost certainly wrong: they are WiL project code, not upstream RTAB-Map code,
and will collide or be lost on any upstream merge. The workspace convention puts
helpers in `script/`. Unresolved.

**The real rig cannot use this path.** `config/wil/cam0_fisheye.yaml` is a
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
