from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can2",
                description="SocketCAN interface: can0, can1, can2, or can3.",
            ),
            DeclareLaunchArgument(
                "motor_id",
                default_value="2",
                description="Motor CAN ID from 1 to 32.",
            ),
            DeclareLaunchArgument(
                "target_degrees",
                description="Absolute multi-turn target angle in degrees; required.",
            ),
            DeclareLaunchArgument(
                "max_speed_dps",
                description="Maximum motor speed in degrees per second; required.",
            ),
            DeclareLaunchArgument(
                "motor_on_before_command",
                default_value="true",
                description="Send motor-on (0x88) immediately before the position command.",
            ),
            DeclareLaunchArgument(
                "timeout_ms",
                default_value="100",
                description="CAN response timeout in milliseconds.",
            ),
            Node(
                package="arm_hardware",
                executable="motor_position",
                name="motor_position",
                output="screen",
                parameters=[
                    {
                        "can_interface": LaunchConfiguration("can_interface"),
                        "motor_id": ParameterValue(
                            LaunchConfiguration("motor_id"), value_type=int),
                        "target_degrees": ParameterValue(
                            LaunchConfiguration("target_degrees"), value_type=float),
                        "max_speed_dps": ParameterValue(
                            LaunchConfiguration("max_speed_dps"), value_type=int),
                        "motor_on_before_command": ParameterValue(
                            LaunchConfiguration("motor_on_before_command"), value_type=bool),
                        "timeout_ms": ParameterValue(
                            LaunchConfiguration("timeout_ms"), value_type=int),
                    }
                ],
            ),
        ]
    )
