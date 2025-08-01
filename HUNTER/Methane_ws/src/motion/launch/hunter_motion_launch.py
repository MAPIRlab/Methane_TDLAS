from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(get_package_share_directory("motion"), "config", "params.yaml")

    return LaunchDescription([
        Node(
            package='motion',
            executable='hunter_motion',
            name='hunter_motion',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[param_file]
        )
    ])