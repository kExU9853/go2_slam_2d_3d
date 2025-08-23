from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # 获取当前包的路径
    pkg_path = os.path.join(os.getcwd(), 'src', 'lidar_time_adapter')
    test_script = os.path.join(pkg_path, 'test', 'test_lidar_time_adapter.py')
    
    return LaunchDescription([
        # 启动主节点
        Node(
            package='lidar_time_adapter',
            executable='lidar_time_adapter_node',
            name='lidar_time_adapter',
            output='screen'
        ),
        
        # 启动测试脚本
        ExecuteProcess(
            cmd=['python3', test_script],
            output='screen'
        )
    ])
