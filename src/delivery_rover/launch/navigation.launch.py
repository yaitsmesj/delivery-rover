# FILE: src/delivery_rover/launch/navigation.launch.py
"""Nav2 for the delivery rover.

Five lifecycle servers plus the manager that walks them to ACTIVE. This
is what nav2_bringup does for a generic robot; writing your own is normal
practice once the robot is yours — you decide which servers exist, and
there is exactly one file to read when something does not come up.

    ros2 launch delivery_rover navigation.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The servers the lifecycle manager owns, in bring-up order.
LIFECYCLE_NODES = [
    "map_server",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description():
    pkg = get_package_share_directory("delivery_rover")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg, "config", "map", "depot.yaml")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Nav2 parameter file"),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Map yaml for map_server"),

        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[params_file, {"yaml_filename": map_yaml}]),

        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[params_file]),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params_file]),

        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params_file]),

        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params_file]),

        # autostart drives every server above from unconfigured to active.
        # bond_timeout 0.0 disables the heartbeat that would otherwise kill
        # servers during long debugging pauses.
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True,
                          "node_names": LIFECYCLE_NODES,
                          "bond_timeout": 0.0}]),
    ])