from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xml_launch = PathJoinSubstitution(
        [FindPackageShare("arm_hardware"), "launch", "motor_slider.launch.xml"]
    )
    return LaunchDescription(
        [IncludeLaunchDescription(AnyLaunchDescriptionSource(xml_launch))]
    )
