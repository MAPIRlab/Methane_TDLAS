import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory("xsens_ros2"), "xsens_params.yaml")
    xsens_driver = Node(
                    package='xsens_ros2',
                    executable='ros2_driver',
                    name='ros2_driver',
                    output='screen',
                    parameters=[params_file]
                )
    
    ld = LaunchDescription()
    ld.add_action(xsens_driver)
    return ld