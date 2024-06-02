from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_path = os.path.join(get_package_share_directory('graph_cpp'), 'maps', 'map.yaml')
    amcl_yaml = os.path.join(get_package_share_directory('graph_cpp'), 'launch', 'amcl.yaml')
    nav2_lifecycle_yaml = os.path.join(get_package_share_directory('graph_cpp'), 'launch', 'amcl.yaml')
    map_server = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'frame_id': 'map'},
                            {'topic_name': 'map'},
                            {'use_sim_time': True},
                            {'yaml_filename': map_path}])

    lifecycle_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': ['map_server']}])




    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]
        )
 
    nav2_lifecycle =Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'node_names' : ["amcl"]},
                        {'autostart': True},
                        {'use_sim_time': True}]
        )

    ld = LaunchDescription()

    # Add the commands to the launch description

    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(amcl_node)
    ld.add_action(nav2_lifecycle)

    return ld
