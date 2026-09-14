"""RTAB-Map as a loop-closure back-end behind VINS or ORB-SLAM3, on the sim bags.

Starts two things and nothing else:

  rtab_camera_shim.py   decompresses the bag's stereo pair and manufactures the
                        camera_info + static TF that the bags do not carry
  rtabmap_slam/rtabmap  the mapper, taking odometry from whichever front-end you
                        are running in another terminal

There is deliberately NO odometry node here. RTAB-Map's own visual odometry
(rtabmap_odom) is the thing we are replacing -- the rtabmap node always takes
its pose from the `odom` topic, so simply not launching an odometry node is
what puts VINS or ORB-SLAM3 in that role. (`visual_odometry:=false` is an
argument of the upstream rtabmap_launch wrapper, not a parameter of this node;
passing it here would do nothing.)

Like every other launch file in this workspace, this one does NOT play a bag.

    # terminal 1   front-end
    ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py
    # terminal 2   this
    ros2 launch ./rtab_sim.launch.py
    # terminal 3   data
    ros2 bag play dataset/static_dataset_loop --rate 0.5

NOTE the missing --clock. These bags RECORD /clock (74180 messages), so adding
--clock makes rosbag2 publish a second, wall-epoch clock on top of it. The two
disagree by ~1.79e9 s, tf2 logs "Detected jump back in time" and clears its
buffer thousands of times, and RTAB-Map never resolves a camera transform.

For ORB-SLAM3 instead, override the two frame/topic arguments:

    ros2 launch ./rtabmap_ros/rtab_sim.launch.py \
        odom_frame:=orbslam3_world body_frame:=base_footprint \
        output_path:=output/output_orb_rtab/simulation/dataset_dynamic_000
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORKSPACE = os.path.expanduser('~/wil_project')
HERE = os.path.dirname(os.path.abspath(__file__))
SHIM = os.path.join(HERE, 'rtab_camera_shim.py')
WRITER = os.path.join(HERE, 'rtab_trajectory_writer.py')
RVIZ_CFG = os.path.join(HERE, 'rtab_map.rviz')
CONFIG_DIR = os.path.join(WORKSPACE, 'vins_fusion_ros2', 'config', 'wil_sim')


def _mode_params(mode, read_only):
    """The ~6 parameters that separate the three modes.

    Everything else in the node is shared, and the shared block is where the
    measured comment blocks live (the Grid/RangeMax sweep, the topic_queue_size
    derivation, the extrinsics-bug history). Forking this file per mode would mean
    three copies of those to drift apart, so the modes are a dict merge instead.

    construct  build a map from nothing. Reproduces the pre-mode defaults exactly.
    extend     open an existing map and add a new session to it. RTAB-Map appends
               by default -- `-d` is purely a file delete -- and the new session is
               stitched to the old one by ordinary appearance-based loop closure.
    localize   read-only against an existing map. No nodes are added; the output is
               map->odom and /localization_pose.
    """
    if mode == 'construct':
        return {'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false'}

    if mode == 'extend':
        return {
            'Mem/IncrementalMemory': 'true',
            # Without this, working memory is initialised with the PREVIOUS
            # session only (Parameters.h:225), so a third session could attach to
            # the second but never to the first. Extending a map means being able
            # to recognise anywhere in it.
            'Mem/InitWMWithAllNodes': 'true',
        }

    # localize
    params = {
        # The master switch: false means Localization mode (Parameters.h:215).
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
        # Publish the grid stored in the database rather than re-assembling it.
        'use_saved_map': True,
        # Do NOT assume we restart at the map origin. Without ground truth the
        # origin is wherever the mapping run's front-end initialised, which is not
        # where this run starts. false = restart from the last saved localization
        # pose, and appearance-based relocalization corrects from there.
        'RGBD/StartAtOrigin': 'false',
        # Anti-teleport. A warehouse is rows of near-identical shelf bays, which is
        # the worst case for perceptual aliasing; this cache is what verifies a
        # candidate against recent odometry instead of accepting any similar-looking
        # place (Parameters.h:403).
        'RGBD/MaxOdomCacheSize': '10',
        # Turns on the LocalizationStatus diagnostic, which only runs when
        # Mem/IncrementalMemory is false. Gives a one-line pass/fail on
        # /diagnostics instead of eyeballing covariances.
        'loc_thr': 0.2,

        # Do not build local occupancy grids while localizing. The upstream
        # default is FALSE; the ROS wrapper forces it true (CoreNode.cpp:45,
        # CoreWrapper.cpp:339) because a mapping node needs it -- but in
        # localization mode /map is served from the database (use_saved_map
        # above), so every grid built here is thrown away.
        #
        # It is not free work. Building a grid runs dense stereo (SGBM) on the
        # pair, measured at 464 ms per keyframe, and that load is what starves
        # VINS: with rtabmap + shim + republish alongside, /imu arrived at 51 Hz
        # against a nominal 60 and VINS diverged to 1595 m in a 14 x 21 m
        # building. Slowing the bag to --rate 0.3 made it WORSE, so the problem
        # is total CPU, not pacing. Feature extraction for loop closure is
        # unaffected -- relocalization still works.
        'RGBD/CreateOccupancyGrid': 'false',
    }
    if read_only:
        # Protects the prebuilt map, but RTAB-Map documents it as incompatible with
        # disjoint sessions in working memory -- a database from a failed `extend`
        # whose sessions never merged will fail to open this way.
        params['Mem/LocalizationReadOnly'] = 'true'
    return params


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).lower()
    if mode not in ('construct', 'extend', 'localize'):
        raise RuntimeError(
            f"mode must be construct|extend|localize, got {mode!r}")
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    body_frame = LaunchConfiguration('body_frame').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context) or body_frame
    ground_height = LaunchConfiguration('ground_height').perform(context)
    optimize_max_error = LaunchConfiguration('optimize_max_error').perform(context)
    read_only = LaunchConfiguration('read_only').perform(context).lower() in (
        'true', '1', 'yes')
    database = os.path.expanduser(LaunchConfiguration('database_path').perform(context))
    output_path = LaunchConfiguration('output_path').perform(context)
    # delete_db defaults to '' meaning "whatever the mode implies": only
    # `construct` starts from nothing. An explicit true on extend/localize is
    # refused rather than honoured -- both exist to work against a map that was
    # expensive to build, and -d would silently erase it before the first frame.
    delete_db_arg = LaunchConfiguration('delete_db').perform(context).lower()
    if delete_db_arg in ('', 'auto'):
        delete_db = (mode == 'construct')
    else:
        delete_db = delete_db_arg in ('true', '1', 'yes')
        if delete_db and mode != 'construct':
            raise RuntimeError(
                f"delete_db:=true would erase the database that mode:={mode} "
                f"exists to read. Use mode:=construct to start fresh.")
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in (
        'true', '1', 'yes')
    always_update = LaunchConfiguration('map_always_update').perform(context).lower() in (
        'true', '1', 'yes')
    wait_tf = float(LaunchConfiguration('wait_for_transform').perform(context))
    grid_range_max = LaunchConfiguration('grid_range_max').perform(context)
    detection_rate = LaunchConfiguration('detection_rate').perform(context)
    shim_rate = LaunchConfiguration('shim_rate').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)

    # database_path must name a FILE. RTAB-Map is launched with -d, which erases
    # whatever that path names before starting, so pointing it at a run directory
    # (easy to do when it looks just like output_path) puts a delete on a
    # directory holding results. Resolve anything that is not a .db to a file
    # INSIDE it, so -d can only ever remove the database itself.
    if os.path.isdir(database) or not database.lower().endswith('.db'):
        database = os.path.join(database, 'rtabmap.db')
        print(f'[rtab_sim] database_path was not a .db file; using {database}')
    os.makedirs(os.path.dirname(database), exist_ok=True)
    print(f'[rtab_sim] mode     = {mode}'
          + ('  (read-only)' if (read_only and mode == 'localize') else ''))
    print(f'[rtab_sim] odom     = TF {odom_frame} -> {body_frame}')
    print(f'[rtab_sim] base     = {base_frame}   ground_height = {ground_height}')
    print(f'[rtab_sim] body     = {body_frame}')
    print(f'[rtab_sim] database = {database}')
    print(f'[rtab_sim] output   = {output_path or "(none -- no vio.csv written)"}')

    shim = ExecuteProcess(
        cmd=['python3', SHIM,
             '--ros-args',
             '-p', 'use_sim_time:=true',
             '-p', f'body_frame:={body_frame}',
             '-p', f'config_dir:={config_dir}',
             '-p', f'rate:={shim_rate}',
             '-p', 'namespace:=/stereo'],
        output='screen')

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        # -d wipes the database at startup. Without it RTAB-Map appends to the
        # previous run's graph and the second run's loop closures are meaningless.
        # Set delete_db:=false to accumulate instead.
        arguments=['-d'] if delete_db else [],
        parameters=[{
            'use_sim_time': True,
            # The robot frame RTAB-Map reports poses in. base_footprint (not the
            # IMU body) is what nav2 and localization_pose must agree on. Grid
            # heights are measured in THIS frame -- see ground_height below.
            'frame_id': base_frame,
            # Take the pose from TF rather than from an odometry TOPIC. Both
            # front-ends broadcast it (vins_estimator.cpp sendTransform, and
            # orbslam3_node's publish_tf, on by default), and it is the only
            # arrangement that syncs reliably here: odometry runs at 40 Hz+
            # while the stereo pair is throttled to ~3 Hz, and folding that
            # into RTAB-Map's 5-topic ApproximateTime set starves it -- it
            # reported "Did not receive data" and kept 3 nodes out of a 45 s
            # run. Using TF leaves only the four image/camera_info topics in
            # the sync, and those share a timestamp exactly. Same run, TF:
            # 28 nodes, 25 neighbour links, 2 loop closures, no warnings.
            # The cost is losing the odometry covariance RTAB-Map would
            # otherwise weight its graph edges with.
            'odom_frame_id': odom_frame,
            'database_path': database,

            'subscribe_stereo': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,

            # The bag's cam0/cam1 stamps differ by ~15 ms and odometry arrives on
            # its own schedule, so nothing here is exactly synchronised.
            'approx_sync': True,
            'sync_queue_size': 30,
            # The per-topic input queue, NOT the sync queue. It must be deep
            # enough to still hold an odometry message when its stereo pair
            # finally shows up: odometry runs at 40 Hz+ while the images are
            # throttled to ~3 Hz, so the default 10 evicts the match in 0.25 s
            # and RTAB-Map reports "Did not receive data since 5 seconds".
            'topic_queue_size': 200,

            # How long to hold an image waiting for the odometry TF that covers
            # its stamp. The camera extrinsics are static, so this is entirely
            # slack for the FRONT-END's latency -- and a front-end that cannot
            # keep up with playback runs a persistent lag, not a jittery one.
            # Measured with VINS at proc_ms ~114 against 100 ms frame spacing
            # (bag --rate 0.5): a steady 83-frame backlog, i.e. a ~4 s offset,
            # against which the old 0.3 s dropped essentially every frame. The
            # images are throttled to 4 Hz with topic_queue_size 200, so there is
            # ~50 s of buffer to wait inside; being generous here costs nothing.
            'wait_for_transform': wait_tf,

            # One loop-closure check per second. The default 1 Hz is already
            # right for a 93 s run at walking pace; raise it for faster motion.
            # Keeps the assembled-map cache even with no subscribers
            # (CoreWrapper.cpp:4180 drops it otherwise). Measured: NOT needed for
            # a late-attaching RViz -- a subscriber joining 32 s into a run still
            # got the full map (492x811 grid, 113k points), because
            # updateMapCaches() rebuilds from Memory. Left available for cases
            # where that rebuild is too slow, not as a fix for late attach.
            'map_always_update': always_update,

            # --- stereo depth quality -------------------------------------
            # Dense stereo defaults to cv::StereoBM, which needs local texture
            # and has none to work with on a painted warehouse floor or a blank
            # wall -- it emits mismatches strung out along the viewing ray, which
            # is what the vertical smears in the cloud are. SGBM is semi-global
            # and far more robust on exactly this kind of surface.
            'Stereo/DenseStrategy': '1',            # 0=StereoBM, 1=StereoSGBM

            # Depth precision is fx*B / disparity, and this rig is fx=640 with a
            # 93.6 mm baseline, so fx*B = 59.9 px*m, so a ONE-pixel disparity
            # error is 0.42 m at 5 m and 1.67 m at 10 m. That argument was used
            # to justify a 4.0 m cap -- but MEASURED against the baked ground
            # truth, the far points are mostly real and cropping them costs more
            # coverage than it buys accuracy:
            #
            #   range   precision   recall     F1   chamfer p95
            #    4 m      94.0 %    44.8 %   60.6      0.30 m
            #    6 m      90.0 %    41.5 %   56.8      0.35 m
            #    8 m      86.0 %    49.8 %   63.1      0.42 m   (mean of 2 runs)
            #   10 m      89.4 %    50.9 %   64.9      0.35 m   <- default
            #   12 m      87.9 %    52.2 %   65.5      0.40 m
            #
            # Caveat: rtabmap-reprocess is NOT deterministic -- it re-runs loop
            # detection and gave 129 vs 133 closures on identical input, worth
            # about 1 F1 point. Differences smaller than that are noise.
            'Grid/RangeMax': grid_range_max,

            # --- ground / obstacle split ----------------------------------
            # By default RTAB-Map decides ground-vs-obstacle from point NORMALS
            # (Grid/NormalsSegmentation) against a 45 deg cone. Normals estimated
            # on noisy stereo depth over a flat, low-texture warehouse floor are
            # meaningless, so the floor itself gets labelled obstacle: measured
            # /cloud_ground 146 points against /cloud_obstacles 76400, which is
            # the scatter of black obstacle cells all over the map.
            #
            # The robot drives a known-flat floor, so a plain height threshold is
            # both simpler and far more robust. The docs say MaxGroundHeight
            # "should be set if NormalsSegmentation is false" -- ground truth puts
            # the body frame ~0.02 m above the floor, so 0.1 m is generous.
            'Grid/NormalsSegmentation': 'false',
            # Grid/MapFrameProjection defaults false, so this threshold is
            # measured in frame_id. Moving frame_id from the IMU body to
            # base_footprint drops the origin by 0.192 m (the SDF puts imu_link at
            # base_footprint + (0.338, 0, 0.192)), so the caller must add that back
            # to keep the measured behaviour. See ground_height.
            'Grid/MaxGroundHeight': ground_height,
            'Grid/MaxObstacleHeight': '2.0',

            # The floor yields NO stereo points (blank, untextured), so nothing
            # is ever labelled ground and the grid would carry obstacles with no
            # free space at all. Ray tracing marks the cells between the sensor
            # and each occupied cell as free, which is what makes the grid a map
            # rather than a scatter of hits.
            'Grid/RayTracing': 'true',

            # Off by default. Drops isolated speckle that survives matching.
            'Grid/NoiseFilteringRadius': '0.1',
            'Grid/NoiseFilteringMinNeighbors': '5',

            # How often a keyframe (graph node) is created. This is not just a
            # cost/detail dial -- it sets how many times each map cell is ever
            # OBSERVED, and therefore whether GridGlobal/OccupancyThr has anything
            # to integrate. Measured at 1.0 Hz on dataset_static_tiny_repeat:
            # 26 nodes, and a median occupied cell was inside range AND inside the
            # 90 deg FOV of only 3 keyframes. Demanding 3 net hits then requires
            # perfect detection every single time. See MEMORY_CLAUDE §6c.
            'Rtabmap/DetectionRate': detection_rate,
            'Mem/IncrementalMemory': 'true',

            # --- keep the map level ---------------------------------------
            # Optimizer/GravitySigma is 0.3 by default, meaning "optimise while
            # PRESERVING gravity orientation" -- but it can only do that if the
            # graph actually has gravity links, and it has none unless RTAB-Map
            # gets an orientation reference. The IMU input is remapped away
            # (above), so the only remaining source is the odometry itself, and
            # that is exactly what Mem/UseOdomGravity is for: "use odometry
            # instead of IMU orientation to add gravity links... we assume that
            # odometry is already aligned with gravity (e.g. VIO)". VINS is
            # stereo-INERTIAL and gravity-aligns during IMU init, so its roll and
            # pitch are absolute and safe to trust here.
            #
            # Left false (the upstream default), the graph carried zero Gravity
            # links and loop-closure optimisation was free to rotate the whole
            # map in roll/pitch -- which is what tilted the occupancy grid out of
            # the horizontal plane and away from the trajectory.
            'Mem/UseOdomGravity': 'true',

            # Reject a loop closure whose optimisation error ratio exceeds this.
            # It is the guard that decides whether a DRIFTING front-end can close
            # a loop at all, and the default 3.0 is tuned for odometry far better
            # than a stereo VIO over a warehouse lap. Measured on
            # dataset_static_tiny_repeat: with ground-truth odometry, 72 closures
            # accepted; with VINS, 36 detected by appearance and ALL rejected here
            # ("Rtabmap.cpp:4033 Rejecting all added loop closures"), giving a map
            # with zero closures. Upstream's own multisession demo raises this to
            # 10 for the same reason.
            'RGBD/OptimizeMaxError': optimize_max_error,
        }] + [_mode_params(mode, read_only)],
        remappings=[
            ('left/image_rect', '/stereo/left/image_rect'),
            ('right/image_rect', '/stereo/right/image_rect'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
            # RTAB-Map's "imu" input is a RELATIVE topic name, so it silently
            # resolves to the bag's /imu and then blocks waiting for a TF to the
            # Gazebo IMU frame (ackermann_robot_001/...) that nothing publishes.
            # Point it at a dead topic: the front-end already fuses the IMU and
            # gravity-aligns its output, so RTAB-Map has no use for it here.
            ('imu', '/rtabmap/imu_unused'),
        ])

    nodes = [shim, rtabmap]

    # Only started when an output_path is given, so a quick exploratory run does
    # not silently overwrite a recorded result.
    if output_path:
        nodes.append(ExecuteProcess(
            cmd=['python3', WRITER,
                 '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', f'output_path:={output_path}',
                 # RTAB-Map's outputs are RELATIVE topic names on a node with no
                 # namespace, so the optimised graph lands at /mapPath, NOT
                 # /rtabmap/mapPath.
                 '-p', 'path_topic:=/mapPath'],
            output='screen'))

    # Stock RViz, no RTAB-Map plugins: /map, /cloud_map and /mapPath are plain
    # OccupancyGrid / PointCloud2 / Path, so rviz_default_plugins renders them.
    # rtabmap_viz and rtabmap_rviz_plugins are NOT built (see MEMORY_CLAUDE) and
    # are not needed for this.
    if use_rviz:
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rtab_rviz',
            arguments=['-d', RVIZ_CFG],
            parameters=[{'use_sim_time': True}],
            output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='construct',
            description='construct = build a new map (-d, starts empty). '
                        'extend = open an existing map and add a session, '
                        'relocalizing into it. localize = read-only against an '
                        'existing map, publishing map->odom and '
                        '/localization_pose. construct reproduces the '
                        'pre-mode behaviour exactly.'),
        DeclareLaunchArgument(
            'read_only', default_value='false',
            description='mode:=localize only. Opens the database read-only so the '
                        'prebuilt map provably cannot change (md5sum it either '
                        'side). Incompatible with a database whose sessions never '
                        'merged.'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Robot frame RTAB-Map reports poses in; empty means use '
                        'body_frame (the pre-mode behaviour). Set base_footprint '
                        'when feeding nav2, so localization_pose and the costmaps '
                        'agree on where the robot is.'),
        DeclareLaunchArgument(
            'optimize_max_error', default_value='3.0',
            description='Reject loop closures whose optimisation error ratio '
                        'exceeds this (0=disabled). 3.0 is the upstream default '
                        'and rejects EVERY closure a drifting VIO front-end '
                        'proposes; upstream multisession uses 10. Raising it '
                        'accepts larger corrections -- and larger mistakes.'),
        DeclareLaunchArgument(
            'ground_height', default_value='0.1',
            description='Grid/MaxGroundHeight, measured in base_frame because '
                        'Grid/MapFrameProjection is false. If base_frame is '
                        'base_footprint rather than the IMU body, add 0.192 (the '
                        'SDF offset) to keep the measured behaviour: 0.29.'),
        DeclareLaunchArgument(
            'odom_frame', default_value='world',
            description='Odometry frame the front-end broadcasts: "world" for '
                        'VINS (world_frame_id), "orbslam3_world" for ORB-SLAM3.'),
        DeclareLaunchArgument(
            'body_frame', default_value='body',
            description='Frame the odometry resolves to: "body" for VINS, '
                        '"base_footprint" for ORB-SLAM3.'),
        DeclareLaunchArgument(
            'output_path', default_value='',
            description='Directory for vio.csv (a path ending in .csv is used as '
                        'given), e.g. output/output_vins_rtab/simulation/'
                        'dataset_dynamic_000. Holds RTAB-Map\'s OPTIMISED graph, '
                        'one pose per node at Rtabmap/DetectionRate -- far sparser '
                        'than the front-ends\' per-frame vio.csv. Empty means do '
                        'not write one.'),
        DeclareLaunchArgument(
            'database_path', default_value='~/output/rtabmap/wil_sim.db',
            description='RTAB-Map database FILE, ending in .db. Anything else is '
                        'treated as a directory and rtabmap.db is created inside '
                        'it -- because the node runs with -d, which ERASES this '
                        'path at startup. Never point it at a results directory. '
                        'Grows with every keyframe; watch disk.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz on /map, /cloud_map and /mapPath. RTAB-Map only '
                        'assembles those when something subscribes, so leaving this '
                        'false costs nothing.'),
        DeclareLaunchArgument(
            'shim_rate', default_value='4.0',
            description='Hz the shim republishes stereo pairs at. This is a HARD '
                        'CAP on detection_rate -- RTAB-Map cannot make a keyframe '
                        'from a frame it never received, so detection_rate above '
                        'this value silently does nothing. Raise both together.'),
        DeclareLaunchArgument(
            'detection_rate', default_value='1.0',
            description='Keyframe rate (Hz). Raising it gives each map cell more '
                        'observations, which is what a multi-hit occupancy gate '
                        'needs. Capped by the shim republish rate (4 Hz). Costs '
                        'database size and vio.csv rows.'),
        DeclareLaunchArgument(
            'grid_range_max', default_value='10.0',
            description='Metres beyond which stereo points are discarded. '
                        'Swept against a baked ground-truth map: recall rises '
                        '45 -> 51 % from 4 m to 10 m while precision only falls '
                        '94 -> 89 %, so the far points are mostly real. Was 4.0, '
                        'chosen to crop smears that turned out to be a camera-TF '
                        'bug (see MEMORY_CLAUDE section 0, fix 13).'),
        DeclareLaunchArgument(
            'wait_for_transform', default_value='5.0',
            description='Seconds to wait for the front-end odometry TF covering '
                        'an image stamp. Raise if the front-end lags playback; '
                        'the real fix is a slower --rate so its backlog drains.'),
        DeclareLaunchArgument(
            'map_always_update', default_value='false',
            description='Keep the assembled-map cache even with no subscribers. '
                        'Measured unnecessary for a late-attaching RViz, which '
                        'still gets the full map; costs CPU/memory every run.'),
        DeclareLaunchArgument(
            'delete_db', default_value='',
            description="Empty (default) means the mode decides: only construct "
                        "wipes. true is REFUSED on extend/localize rather than "
                        "honoured, because -d would erase the map they exist to "
                        "read. false forces append even in construct."),
        DeclareLaunchArgument(
            'config_dir', default_value=CONFIG_DIR,
            description='Directory holding cam0_pinhole.yaml, cam1_pinhole.yaml, '
                        'stereo_imu.yaml.'),
        OpaqueFunction(function=launch_setup),
    ])
