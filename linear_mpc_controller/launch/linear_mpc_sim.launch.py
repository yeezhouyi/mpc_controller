# Launch the standalone tracking chain (WSL2 / Gazebo Harmonic).
#
#   ros2 launch linear_mpc_controller linear_mpc_sim.launch.py \
#       world:=tracking_empty.sdf headless:=True use_sim_time:=True
#
# NOTE: the Gazebo/TurtleBot3 bringup is environment-specific; this file
# wires the controller nodes and parameters.  Full TurtleBot3 integration is
# the U5/C3 acceptance item (not yet exercised on this machine).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration("world", default="tracking_empty.sdf")
    headless = LaunchConfiguration("headless", default="True")
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="tracking_empty.sdf"),
            DeclareLaunchArgument("headless", default_value="True"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            # Gazebo world (comment in per-environment command; U5 gate):
            ExecuteProcess(
                cmd=[
                    "gz", "sim",
                    "-r", "-s",
                    "--headless-rendering" if False else "",
                    world,
                ],
                output="screen",
            ),
            Node(
                package="linear_mpc_controller",
                executable="linear_mpc_node",
                name="linear_mpc_node",
                output="screen",
                parameters=["config/linear_mpc_params.yaml"],
                remappings=[],
            ),
            Node(
                package="linear_mpc_controller",
                executable="velocity_arbiter_node",
                name="velocity_arbiter",
                output="screen",
                parameters=["config/velocity_arbiter.yaml"],
            ),
        ]
    )
