import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Falcon TDLAS Methane detector
    TDLAS_methane_falcon = [
        Node(
            package='falcon_tdlas',
            executable='falcon_tdlas',
            name='falcon_tdlas',            
            output='screen',
            prefix="xterm -hold -e",
            parameters=[{
                "port": "/dev/falcon_tdlas",
                "topic": "/falcon/reading",
                "frequency": 10.0,
                "verbose": True                
            }]   
        ),
    ]    
    

    actions=[]
    actions.extend(TDLAS_methane_falcon)    
    return[
        GroupAction
        (
            actions=actions
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        OpaqueFunction(function = launch_setup)
    ])