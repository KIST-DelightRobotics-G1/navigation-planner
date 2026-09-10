# One-shot bringup for the external LIO engine: starts the Livox Mid-360 driver AND
# FAST-LIO in a single `ros2 launch`, using OUR config (next to this file). This runs
# inside the pinned ros:humble container (see docs/LIO_ENGINE.md); the planner consumes
# its DDS output (/Odometry_loc, /cloud_registered_1) ROS-free. We orchestrate the
# already-built upstream nodes here — no upstream source is modified or vendored.
#
#   ros2 launch /root/lio_engine/lio_bringup.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    mid360_json = os.path.join(here, 'config', 'MID360_config.json')  # roll=180, IPs
    fastlio_yaml = os.path.join(here, 'config', 'mid360.yaml')        # lid_topic=/livox/lidar

    # Livox Mid-360 driver — CustomMsg on /livox/lidar, IMU on /livox/imu.
    driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},        # 1 = Livox CustomMsg (what FAST-LIO lidar_type:1 wants)
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'user_config_path': mid360_json},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    # FAST-LIO — publishes T_odom_lidar on /Odometry_loc + registered cloud on
    # /cloud_registered_1 (frame camera_init = odom).
    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[fastlio_yaml, {'use_sim_time': False}],
    )

    return LaunchDescription([driver, fastlio])
