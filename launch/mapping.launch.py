import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('warehouse_robot')
    
    # Launch Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 
                          'rviz_config': os.path.join(pkg_path, 'config', 'mapping.rviz')}.items()
    )

    # SLAM Toolbox
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}]
    )

    return LaunchDescription([
        simulation,
        slam_toolbox
    ])
