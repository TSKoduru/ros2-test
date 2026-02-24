from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        description='Map file name (without .yaml extension)'
    )

    pose_source_arg = DeclareLaunchArgument(
        'pose_source',
        default_value='manual',
        description='Pose source: "manual" for RViz clicks, "tf" for localization node'
    )

    map_file_path = PathJoinSubstitution([
        get_package_share_directory('mbot_nav'),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path, 'use_sim_time': False}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    navigation_node = Node(
        package='mbot_nav',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'pose_source': LaunchConfiguration('pose_source')}]
    )

    return LaunchDescription([
        map_name_arg,
        pose_source_arg,
        map_server,
        lifecycle_manager,
        navigation_node,
    ])