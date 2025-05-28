from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import os

viz_arg = DeclareLaunchArgument("rviz", default_value="false")

config = os.path.join(
        get_package_share_directory('cloud_regions_cpp'),
        'config',
        'params.yaml'
        )

rviz_config = os.path.join(
        get_package_share_directory('cloud_regions_cpp'),
        'rviz',
        'regions.rviz'
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
    
def rviz():
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    
    
def generate_launch_description():
    return LaunchDescription([
        viz_arg,
        regions(),
        rviz(),
    ])