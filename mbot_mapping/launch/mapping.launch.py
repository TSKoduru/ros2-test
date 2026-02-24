from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform: map -> odom (identity)
        # In odometry-based mapping, the map frame and odom frame always overlap
        # since there is no drift correction.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen'
        ),

        # Launch the mapping node with grid and publishing parameters
        Node(
            package='mbot_mapping',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
        ),
    ])

