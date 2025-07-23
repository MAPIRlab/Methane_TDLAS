#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('percepcion')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='percepcion',          # tu paquete
            executable='mov_PTU',          # coincide con add_executable(mov_PTU …)
            name='mov_PTU',
            output='screen',
            parameters=[params_file],      # ruta absoluta
        ),
    ])