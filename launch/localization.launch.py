import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('warehouse_robot')
    
    # Use the source path for the map to avoid rebuild issues during testing
    user_home = os.path.expanduser('~')
    default_map_path = os.path.join(user_home, 'gazebo_ros_ws', 'src', 'warehouse_robot', 'maps', 'warehouse_map.yaml')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    
    map_file = LaunchConfiguration('map')
    params_file = os.path.join(pkg_path, 'config', 'localization_params.yaml')

    # Launch Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 
                          'rviz_config': os.path.join(pkg_path, 'config', 'localization.rviz')}.items()
    )

    # Localization Nodes (Map Server + AMCL)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        map_file_arg,
        simulation,
        map_server,
        amcl,
        lifecycle_manager
    ])
