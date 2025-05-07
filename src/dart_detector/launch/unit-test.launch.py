import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    config_file = os.path.join(
        os.getenv('HOME'),
        'dart-ros2-workspace',
        'src',
        'dart_detector',
        'config',
        'node_launcher_detector.yaml'
    )

    # 启动 lifecycle 节点
    lifecycle_node = Node(
        package='dart_detector',
        executable='node_launcher_detector',
        output='screen',
        parameters=[config_file]
    )

    # 定时调用 lifecycle 命令，先 transition 到 'configure'
    configure_transition = TimerAction(
        period=2.0,  # 延时 2 秒后执行
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/dart_launcher_detector', 'configure'],
            output='screen'
        )]
    )

    # 再 transition 到 'activate'
    activate_transition = TimerAction(
        period=5.0,  # 延时 5 秒后执行
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/dart_launcher_detector', 'activate'],
            output='screen'
        )]
    )

    return LaunchDescription([
        lifecycle_node,
        configure_transition,
        activate_transition
    ])