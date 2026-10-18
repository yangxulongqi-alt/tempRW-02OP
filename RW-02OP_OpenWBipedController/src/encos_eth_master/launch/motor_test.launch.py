
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_node = Node(
        package='encos_eth_master',
        executable='motor_test_basic',
        name='motor_test_basic',
        output='screen',
        parameters=[   # 参数字典
            {'mode': 1},        # 1-单电机测试
            {'motor_id', 20},  # 电机ID
        ],
        remappings=[
            # 针对话题、服务、动作的名称映射
        ]
    )

    return LaunchDescription([
        controller_node,
    ])