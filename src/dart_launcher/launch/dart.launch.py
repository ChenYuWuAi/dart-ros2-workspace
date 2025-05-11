# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os


def generate_launch_description():
    # 获取配置文件路径
    config_file_path = os.path.join(
        os.getenv('PROJECT_DIR', '/home/devcontainers/dart-ros2-workspace'),
        'src/dart_launcher/config/node_dart_param_gateway.yaml'
    )

    # 定义节点
    node_dart_param_gateway = Node(
        package='dart_launcher',
        executable='node_dart_param_gateway',
        name='node_dart_param_gateway',
        parameters=[config_file_path],
        output='screen'
    )

    # 定义生命周期控制
     # 定时调用 lifecycle 命令，先 transition 到 'configure'
    configure_transition = TimerAction(
        period=2.0,  # 延时 2 秒后执行
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/node_dart_param_gateway', 'configure'],
            output='screen'
        )]
    )

    # 再 transition 到 'activate'
    activate_transition = TimerAction(
        period=5.0,  # 延时 5 秒后执行
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/node_dart_param_gateway', 'activate'],
            output='screen'
        )]
    )

    # 创建LaunchDescription
    launch_description = LaunchDescription([
        node_dart_param_gateway,
        configure_transition,
        activate_transition
    ])
    return launch_description
