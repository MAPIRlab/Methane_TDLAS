import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al launch de la PTU
    ptu_pkg = get_package_share_directory('interbotix_xsturret_control')
    ptu_launch_file = os.path.join(ptu_pkg, 'launch', 'xsturret_control_launch.py')

    # Se lanza la PTU normalmente
    ptu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ptu_launch_file)
    )

    # Ejecutar el nodo de tracking con ros2 run
    tracking_node = ExecuteProcess(
        cmd=[
            'xterm',
            '-T', 'Tracking',
            '-e', 'ros2', 'run', 'percepcion', 'mov_PTU'
        ],
        output='screen'
    )

    return LaunchDescription([
        ptu_launch,
        tracking_node
    ])