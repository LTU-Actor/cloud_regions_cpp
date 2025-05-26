from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

config = os.path.join(
        get_package_share_directory('cloud_regions_cpp'),
        'config',
        'params.yaml'
        )

def regions():
    return Node(
        package='cloud_regions_cpp',
        executable='regions',
        name='regions',
        output='screen',
        parameters=[
            config
            ],
    )
    
    
def generate_launch_description():
    return LaunchDescription([
        regions()
    ])