# FILE: src/delivery_rover/launch/perception.launch.py
"""Camera to detections.

ros2 launch delivery_rover perception.launch.py
ros2 launch delivery_rover perception.launch.py source:=assets/feed.mp4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="0",
                description="Camera index as a string, or a path to a video file",
            ),
            DeclareLaunchArgument("model", default_value="yolo11n.pt"),
            DeclareLaunchArgument(
                "device",
                default_value="",
                description="Empty picks automatically: mps, cuda, else cpu",
            ),
            DeclareLaunchArgument("rate_hz", default_value="5.0"),
            Node(
                package="delivery_rover",
                executable="vision",
                name="robot_vision_brain",
                output="screen",
                parameters=[
                    {
                        "source": ParameterValue(
                            LaunchConfiguration("source"), value_type=str
                        ),
                        "model": ParameterValue(
                            LaunchConfiguration("model"), value_type=str
                        ),
                        "device": ParameterValue(
                            LaunchConfiguration("device"), value_type=str
                        ),
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
