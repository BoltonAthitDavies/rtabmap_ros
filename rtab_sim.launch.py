"""MOVED (2026-11-04). The RTAB-Map launch, the stereo shim, the trajectory
writer and the RViz config now live in the vslam_navigation package:

    vslam_navigation/launch/rtabmap.launch.py
    vslam_navigation/scripts/rtab_camera_shim.py
    vslam_navigation/scripts/rtab_trajectory_writer.py
    vslam_navigation/rviz/rtab_map.rviz

They were WiL project files sitting at the root of this fork, which is upstream
code pinned to one commit (design record, Section 8). This stub keeps the old
command working:

    ros2 launch ./rtabmap_ros/rtab_sim.launch.py <same arguments>

is now the same as

    ros2 launch vslam_navigation rtabmap.launch.py <same arguments>

Launch arguments given on the command line are inherited by the included
description, so nothing needs forwarding here. Requires the workspace install
to be sourced (colcon build --symlink-install --packages-select vslam_navigation).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    target = os.path.join(get_package_share_directory('vslam_navigation'),
                          'launch', 'rtabmap.launch.py')
    return LaunchDescription([
        LogInfo(msg='[rtab_sim] rtab_sim.launch.py moved to vslam_navigation; '
                    'use: ros2 launch vslam_navigation rtabmap.launch.py ...'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target)),
    ])
