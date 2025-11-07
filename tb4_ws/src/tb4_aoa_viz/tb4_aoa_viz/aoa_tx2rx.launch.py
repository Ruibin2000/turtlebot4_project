from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb4_aoa_viz',
            executable='aoa_tx2rx_node',
            name='aoa_tx2rx_node',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'aoa_topic': '/aoa_angle'},
                {'publish_rate_hz': 10.0},
                # 方案一：使用固定 TX（map 坐标）
                {'use_tx_topic': False},
                {'tx_x': 2.5},
                {'tx_y': -1.0},
                # 方案二（如需）：订阅 /tx_pose
                # {'use_tx_topic': True},
                # {'tx_pose_topic': '/tx_pose'},
            ]
        )
    ])
