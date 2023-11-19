import launch
import launch_ros


def generate_launch_description():
    # remove namespace ('/') from tf, so tf topics from custom namespace will be used
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    ld = launch.LaunchDescription()
    collision = launch_ros.actions.Node(
        package="scenario_lane_change",
        executable="scenario_lane_change",
        remappings=remappings,
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ld.add_action(collision)
    return ld
