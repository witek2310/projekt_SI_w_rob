from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    map_path = os.path.join(get_package_share_directory('graph_cpp'), 'maps', 'map.yaml')


    # publish occupancy grid
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



    #turn gazebo on, spawn burger

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('graph_cpp'),
        'maps',
        'word_without_robot.world'
    )

    print(world)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    # print('urdf_file_name : {}'.format(urdf_file_name))

    # urdf_path = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'urdf',
    #     urdf_file_name)

    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()

    # robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': robot_desc
    #     }],
    # )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)

    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)


    return ld
