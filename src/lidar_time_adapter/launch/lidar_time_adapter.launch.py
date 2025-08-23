from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_time_adapter',
            executable='lidar_time_adapter_node',
            name='lidar_time_adapter',
            output='screen',
            parameters=[{
                'remove_nan_points': True,
                'publish_clean_topic': False,
                'clean_topic_name': '/lidar_points_clean'
            }]
        )
    ])
