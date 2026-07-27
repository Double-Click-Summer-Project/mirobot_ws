"""Launch one minimal WLKATA Mirobot driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("mirobot_driver"), "config", "mirobot.yaml"]
    )

    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")
    baud_rate = LaunchConfiguration("baud_rate")
    dry_run = LaunchConfiguration("dry_run")
    auto_home = LaunchConfiguration("auto_home")
    enable_motion_after_auto_home = LaunchConfiguration(
        "enable_motion_after_auto_home"
    )
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="mirobot",
                description="ROS namespace for all driver interfaces",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Mirobot serial device",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="115200",
                description="Mirobot serial baud rate",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Log generated G-code without opening serial",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="false",
                description=(
                    "Start homing as soon as the serial connection is ready"
                ),
            ),
            DeclareLaunchArgument(
                "enable_motion_after_auto_home",
                default_value="false",
                description=(
                    "Keep joint/XYZ commands enabled after automatic "
                    "homing completes"
                ),
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Driver parameter YAML",
            ),
            Node(
                package="mirobot_driver",
                executable="driver",
                namespace=namespace,
                name="driver",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config,
                    {
                        "port": port,
                        "baud_rate": ParameterValue(
                            baud_rate,
                            value_type=int,
                        ),
                        "dry_run": ParameterValue(
                            dry_run,
                            value_type=bool,
                        ),
                        "auto_home": ParameterValue(
                            auto_home,
                            value_type=bool,
                        ),
                        "enable_motion_after_auto_home": ParameterValue(
                            enable_motion_after_auto_home,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
