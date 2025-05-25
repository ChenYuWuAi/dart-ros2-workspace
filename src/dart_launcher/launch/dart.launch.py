# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os


def generate_launch_description():
    # 获取配置文件路径
    home_path = os.getenv("HOME")
    if home_path is None:
        raise EnvironmentError("HOME environment variable is not set.")

    # 定义节点
    node_dart_param_gateway = Node(
        package="dart_launcher",
        executable="node_dart_param_gateway",
        parameters=[
            os.path.join(
                home_path,
                "dart-ros2-workspace/src/dart_launcher/config/node_dart_param_gateway.yaml",
            )
        ],
        output="screen",
    )

    node_dart_launcher_detector = Node(
        package="dart_detector",
        executable="node_dart_launcher_detector",
        parameters=[
            os.path.join(
                home_path,
                "dart-ros2-workspace/src/dart_detector/config/node_dart_launcher_detector.yaml",
            )
        ],
        output="screen",
    )

    node_dart_launcher_lifecycle = Node(
        package="dart_launcher",
        executable="node_dart_launcher_lifecycle",
        parameters=[
            os.path.join(
                home_path,
                "dart-ros2-workspace/src/dart_launcher/config/node_dart_launcher_lifecycle.yaml",
            )
        ],
        output="screen",
    )

    node_dart_app = Node(
        package="dart_launcher",
        executable="node_dart_app",
        parameters=[
            os.path.join(
                home_path,
                "dart-ros2-workspace/src/dart_launcher/config/node_dart_app.yaml",
            )
        ],
        output="screen",
    )

    # # 定时调用 lifecycle 命令，先 transition 到 'configure'
    # configure_dart_launcher_detector_transition = TimerAction(
    #     period=4.0,  # 延时 4 秒后执行
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 "ros2",
    #                 "lifecycle",
    #                 "set",
    #                 "/dart_launcher_detector",
    #                 "configure",
    #             ],
    #             output="screen",
    #         )
    #     ],
    # )

    # # 再 transition 到 'activate'
    # activate_dart_launcher_detector_transition = TimerAction(
    #     period=8.0,  # 延时 8 秒后执行
    #     actions=[
    #         ExecuteProcess(
    #             cmd=["ros2", "lifecycle", "set", "/dart_launcher_detector", "activate"],
    #             output="screen",
    #         )
    #     ],
    # )

    # configure_node_dart_param_gateway_transition = TimerAction(
    #     period=2.0,  # 延时 2 秒后执行
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 "ros2",
    #                 "lifecycle",
    #                 "set",
    #                 "/node_dart_param_gateway",
    #                 "configure",
    #             ],
    #             output="screen",
    #         )
    #     ],
    # )

    # activate_node_dart_param_gateway_transition = TimerAction(
    #     period=5.0,  # 延时 5 秒后执行
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 "ros2",
    #                 "lifecycle",
    #                 "set",
    #                 "/node_dart_param_gateway",
    #                 "activate",
    #             ],
    #             output="screen",
    #         )
    #     ],
    # )

    # 创建LaunchDescription
    launch_description = LaunchDescription(
        [
            node_dart_param_gateway,
            node_dart_launcher_detector,
            node_dart_launcher_lifecycle,
            node_dart_app,
            # configure_dart_launcher_detector_transition,
            # activate_dart_launcher_detector_transition,
            # configure_node_dart_param_gateway_transition,
            # activate_node_dart_param_gateway_transition,
        ]
    )
    return launch_description
