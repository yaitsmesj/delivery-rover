# FILE: src/delivery_rover/launch/robot.launch.py
"""The rover itself — the base and nothing else.

On real hardware this is the file that would start motor drivers, the
lidar driver and robot_state_publisher. Here it starts the simulated
base, which speaks the same topics. Everything above it (Nav2, the
mission) is unchanged by that swap — which is the point.

    ros2 launch delivery_rover robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start = LaunchConfiguration("start")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start", default_value="home",
            description="Named location from world.LOCATIONS to spawn at"),

        Node(package="delivery_rover", executable="sim",
             name="diff_drive_sim", output="screen",
             parameters=[{"start": start}]),
    ])