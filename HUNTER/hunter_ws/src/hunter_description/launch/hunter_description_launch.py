import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_dir = get_package_share_directory("hunter_description")
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default_rviz1.rviz')
    
    urdf_hunter_file = os.path.join(pkg_dir,'description','robot.urdf.xacro')
    
    launch_dir = os.path.join(pkg_dir,'launch') 
    namespace = "hunter20"
    
    ############################################
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RVIZ')
    
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time')
    
    
    # Launch rviz2 with a predefined configuration file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_file],
        output="screen",
        condition=IfCondition(use_rviz),
        # prefix="terminator -x"
    )

   
    robot_description_config = Command(['xacro ', urdf_hunter_file, ' sim_mode:=', use_sim_time])
    # robot_description_config for hunter and robot_description_turtlebot for turtlebot
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    # Launch Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        parameters=[params],
        )
    
    
    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', "hunter", 
                        '-topic', 'robot_description',
                        '-z', "10",],
                        output="screen"
                        )
     
    ack_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"],
        output="screen"
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )
    
    
    ld = LaunchDescription()
    
    #launch declares
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_sim_time_cmd)
    
    ld.add_action(rsp)
    #launch simulators
    # ld.add_action(gz_client)
    
    #launch Rviz
    ld.add_action(rviz)
    
    #launch spawners
    # ld.add_action(spawn_entity)
    # ld.add_action(ack_cont_spawner)
    # ld.add_action(joint_broad_spawner)
    
    
    return ld

    