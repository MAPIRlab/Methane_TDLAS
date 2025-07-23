import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Ruta al archivo de parámetros de Nav2
    nav2_params_path = os.path.join(
        get_package_share_directory("hunter_navigation"),
        "config",
        "navigation_params.yaml",
    )
    # Ruta al archivo de configuración de RViz
    rviz_config_path = os.path.join(
        get_package_share_directory("hunter_navigation"),
        "rviz",
        "rviz_config.rviz",
    )
    # Ruta al archivo de configuración para mandar las poses
    config_file = os.path.join(
        get_package_share_directory("hunter_navigation"),
        "config",
        "poses_params.yaml",
    )

    return LaunchDescription(
        [
            # Nodo para pasarle las poses
            Node(
                package='hunter_navigation',
                executable='navigation_through_poses',
                name='navigation_through_poses',
                output='screen',
                prefix="xterm -hold -e",
                parameters=[config_file]),
            # Nodo para pintar las lineas
            Node(
                package='interbotix_methane_pkg',
                executable='methane_visualizer',
                name='methane_visualizer',
                output='screen',
                prefix="xterm -hold -e"),
            # Nodo de simulación RVIz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                emulate_tty=True,
                prefix="xterm -hold -e"),
            #  Nodo de mapa
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            #   Nodo de mapa amcl
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_amcl",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
                remappings=[
                ('/map', '/map_amcl')
            ]
            ),
            # Nodo de localización (AMCL)
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo del planner global
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo del controlador de movimiento
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo de comportamiento basado en árboles de decisión (BT Navigator)
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo de behaviors de Nav2
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Lifecycle Manager para control de nodos de Nav2
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                            "map_server_amcl",
                            "planner_server",
                            "controller_server",
                            "behavior_server",
                            "bt_navigator",
                            #"collision_monitor",
                        ],
                    }
                ],
            ),
        ]
    )