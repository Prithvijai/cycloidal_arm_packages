import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # to launch the rsp.launch.py file via this launch file 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cycloidal_arm'),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time':'true'}.items()
    )
    
    # To Launch The gazebo simulator 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
    )

    # to spawn to robotic arm in the gazebo

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic','robot_description','-entity','cycloidal_arm'],
                        output='screen')

    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity
        
        
    ])