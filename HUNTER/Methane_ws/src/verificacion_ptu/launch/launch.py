from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    param_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        Node(
            package='verificacion_ptu',
            executable='comandos_velocidad',
            name='comandos_velocidad',
            output='screen',
            prefix="xterm -hold -e",
            parameters=[param_file]
        )
    ])
