# FILE: src/delivery_rover/launch/bringup.launch.py
"""The whole robot, one command.

    ros2 launch delivery_rover bringup.launch.py

Composition, not duplication: this file includes the other launch files
rather than restating their nodes, so `ros2 launch delivery_rover
navigation.launch.py` on its own keeps working and there is one place to
change any given node. Ctrl-C here stops everything it started.

Useful switches:
    perception:=false     bring up the base and Nav2 only
    mission:=false        no mission node; drive it yourself
    viz:=true             also start the Foxglove bridge (needs cbor2)
    auto_start:=true      run one delivery as soon as Nav2 is ready
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("delivery_rover")
    launch_dir = os.path.join(pkg, "launch")

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("mission", default_value="true"),
        DeclareLaunchArgument("viz", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("cargo", default_value="bottle"),
        DeclareLaunchArgument("source", default_value="0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg, "config", "nav2_params.yaml"),
            description="Override to try a tuning change without editing "
                        "the installed file"),

        include("robot.launch.py"),
        include("navigation.launch.py", launch_arguments={
            "params_file": LaunchConfiguration("params_file")}.items()),

        GroupAction(
            condition=IfCondition(LaunchConfiguration("perception")),
            actions=[include(
                "perception.launch.py",
                launch_arguments={
                    "source": LaunchConfiguration("source")}.items())]),

        Node(package="delivery_rover", executable="mission",
             name="mission", output="screen",
             condition=IfCondition(LaunchConfiguration("mission")),
             parameters=[{"cargo": LaunchConfiguration("cargo"),
                          "auto_start": LaunchConfiguration("auto_start")}]),

        # Foxglove connects to this: Open connection -> Rosbridge ->
        # ws://localhost:9090. foxglove_bridge is Linux-only on RoboStack,
        # so the Mac path is rosbridge; the layout is identical either way.
        Node(package="rosbridge_server", executable="rosbridge_websocket",
             name="rosbridge_websocket", output="screen",
             condition=IfCondition(LaunchConfiguration("viz")),
             parameters=[{"port": 9090}]),
    ])