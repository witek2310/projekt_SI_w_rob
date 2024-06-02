from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    amcl_yaml = os.path.join(get_package_share_directory('graph_cpp'), 'launch', 'amcl.yaml')
    nav2_lifecycle_yaml = os.path.join(get_package_share_directory('graph_cpp'), 'launch', 'amcl.yaml')

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


    ld.add_action(amcl_node)
    ld.add_action(nav2_lifecycle)

    return ld
