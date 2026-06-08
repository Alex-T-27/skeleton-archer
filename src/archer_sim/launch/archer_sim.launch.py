"""
P2.1 launch: open Gazebo with the archer world and bridge the two joint-command
topics from ROS 2 to Gazebo.

After launching, drive the archer from another terminal:
    ros2 topic pub /archer/base_cmd std_msgs/msg/Float64 "{data: 0.7}"
    ros2 topic pub /archer/draw_cmd std_msgs/msg/Float64 "{data: -0.15}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_archer = get_package_share_directory('archer_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_archer, 'worlds', 'archer_world.sdf')

    # Start Gazebo (gz sim) running the world. "-r" starts it un-paused.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Bridge: ROS 2 std_msgs/Float64  -->  Gazebo gz.msgs.Double
    # The "]" means ROS -> GZ (we publish from ROS, the joint controller listens in GZ).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/archer/base_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/archer/draw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    return LaunchDescription([gz_sim, bridge])
