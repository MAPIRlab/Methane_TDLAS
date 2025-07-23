import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Ruta al launch de tu robot Hunter
    hunter_pkg = get_package_share_directory('hunter_description')
    hunter_launch_file = os.path.join(hunter_pkg, 'launch', 'hunter_description_launch.py')

    # AGILEX HUNTER BASE
    hunter_base_pkg = get_package_share_directory("hunter_base")
    hunter_base_launch = os.path.join(hunter_base_pkg, 'launch', 'hunter_base.launch.py')

    # Ruta al launch de Ouster
    ouster_pkg = get_package_share_directory('ouster_ros')
    ouster_launch_file = os.path.join(ouster_pkg, 'launch', 'sensor.launch.xml')

    # Ruta al launch del TDLAS
    tdlas_pkg = get_package_share_directory('falcon_tdlas')
    tdlas_launch_file = os.path.join(tdlas_pkg, 'launch', 'falcon_launch.py')

    # Lanzar hunter_description en xterm con título personalizado
    hunter_launch_xterm = ExecuteProcess(
        cmd=['xterm', '-T', 'HUNTER_DESCRIPTION', '-e', 'ros2', 'launch', 'hunter_description', 'hunter_description_launch.py'],
        output='screen'
    )

    # Lanzar ouster_ros en xterm con título personalizado
    ouster_launch_xterm = ExecuteProcess(
        cmd=['xterm', '-T', 'OUSTER_SENSOR', '-e', 'ros2', 'launch', 'ouster_ros', 'sensor.launch.xml', 'sensor_hostname:=10.5.5.100'],
        output='screen'
    )

    # Incluir hunter_base (sin xterm)
    hunter_base_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hunter_base_launch)
    )

    # Incluir tdlas (sin xterm)
    TDLAS_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tdlas_launch_file),
    )

    return LaunchDescription([
        hunter_launch_xterm,
        ouster_launch_xterm,
        hunter_base_include,
        TDLAS_launch
    ])
