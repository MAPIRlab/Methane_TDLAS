import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al launch del TDLAS
    falcon_tdlas_pkg = get_package_share_directory('falcon_tdlas')
    falcon_launch_file = os.path.join(falcon_tdlas_pkg, 'launch', 'falcon_launch.py')

    # Ruta al launch del puntero laser
    puntero_pkg = get_package_share_directory('lightwarelidar2')
    puntero_launch_file = os.path.join(puntero_pkg, 'launch', 'lightwarelidar_launch.py')

    # Ruta al launch del GPS
    gps_pkg = get_package_share_directory('nmea_navsat_driver')
    gps_launch_file = os.path.join(gps_pkg, 'launch', 'nmea_serial_driver.launch.py')

    # Se lanza el TDLAS normalmente
    tdlas_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(falcon_launch_file)
    )

    # Se lanza el puntero laser 
    puntero_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(puntero_launch_file)
    )

    # Se lanza el GPS en xterm
    gps_launch = ExecuteProcess(
        cmd=[
            'xterm',
            '-T', 'DeluoGPS',  # Título de la ventana
            '-e', 'ros2', 'launch', 'nmea_navsat_driver', 'nmea_serial_driver.launch.py'
        ],
        output='screen'
    )


    return LaunchDescription([
        tdlas_launch,
        gps_launch,
        puntero_launch
    ])
