import os
import launch
import launch_ros
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription()
    spec_lane_change_node = launch_ros.actions.Node(
        package="spec_lane_change",
        executable="spec_lane_change_node",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    ld.add_action(spec_lane_change_node)

    return ld
