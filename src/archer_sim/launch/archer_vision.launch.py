"""
P2.3 launch: the perception half of the loop.

Brings up Gazebo with the vision world (archer + camera + AprilTag target),
bridges the joint commands AND the camera image into ROS 2, and starts the
target_detector node.

This does NOT start the sequencer, so you can first verify detection alone:
    ros2 topic echo /archer/target_angle      # should show ~ +30
Then close the loop in another terminal:
    ros2 run archer_sim sequencer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_archer = get_package_share_directory('archer_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_archer, 'worlds', 'archer_vision_world.sdf')

    # Gazebo's camera sensor renders off-screen via EGL. On this hybrid-GPU
    # machine libglvnd otherwise routes that through mesa/dri2 on the NVIDIA
    # device and fails (blank gradient image). Pin EGL to the NVIDIA vendor so
    # the sensor renders on the real GPU.
    force_nvidia_egl = SetEnvironmentVariable(
        '__EGL_VENDOR_LIBRARY_FILENAMES',
        '/usr/share/glvnd/egl_vendor.d/10_nvidia.json')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Joint command bridge (ROS -> GZ), same as P2.1.
    joint_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/archer/base_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/archer/draw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # Camera image bridge (GZ -> ROS): /archer/camera as sensor_msgs/Image.
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/archer/camera'],
        output='screen',
    )

    detector = Node(
        package='archer_sim',
        executable='target_detector',
        output='screen',
    )

    return LaunchDescription(
        [force_nvidia_egl, gz_sim, joint_bridge, image_bridge, detector])
