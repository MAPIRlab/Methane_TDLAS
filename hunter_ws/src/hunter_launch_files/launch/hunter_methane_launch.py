from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta la paquete de la base del hunter
    hunter_base_pkg = get_package_share_directory("hunter_base")
    hunter_base_launch = os.path.join(hunter_base_pkg, 'launch', 'hunter_base.launch.py')

    # Ruta al paquete del GPS
    nmea_pkg = get_package_share_directory("nmea_navsat_driver")
    nmea_launch_file = os.path.join(nmea_pkg, 'launch', 'nmea_serial_driver.launch.py')
    
    # Ruta al paquete del movimiento del hunter
    motion_pkg = get_package_share_directory("motion")
    motion_launch_file = os.path.join(motion_pkg, 'launch', 'hunter_motion_launch.py')

    # Ruta al paquete de MQTT
    mqtt_pkg = get_package_share_directory("mqtt_bridge")
    mqtt_launch_file = os.path.join(mqtt_pkg, 'launch', 'mqtt_basic_robot_launch.py')
    
    # Lanzar el brazo (con xterm)
    arm_launch_xterm = ExecuteProcess(
        cmd=['xterm', '-T', 'Interbotix_arm', '-e', 'ros2', 'launch', 'interbotix_xsarm_moveit', 'xsarm_moveit_methane.launch.py', 'robot_model:=wx250'],
        output='screen'
    )

    # Lanzar el adaptador (con xterm)
    adaptador_launch_xterm = ExecuteProcess(
        cmd=['xterm', '-T', 'Adaptador', '-e', 'ros2', 'run', 'MQTT_adapter', 'adaptador'], output='screen'
    )
    # Incluir otros launch
    hunter_base_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hunter_base_launch)
    )
    nmea_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nmea_launch_file)
    )
    motion_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_launch_file)
    )
    mqtt_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mqtt_launch_file)
    )

    # Agrupar todo bajo /hunter
    return LaunchDescription([
        GroupAction([
            mqtt_include,
            PushRosNamespace('hunter'),
            hunter_base_include,
            nmea_include,
            arm_launch_xterm, 
            adaptador_launch_xterm,
            motion_include
        ])
    ])
