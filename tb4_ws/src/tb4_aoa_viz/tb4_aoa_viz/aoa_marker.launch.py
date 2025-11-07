from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb4_aoa_viz',
            executable='aoa_marker_node',
            name='aoa_marker_node',
            output='screen',
            parameters=[{
                # --- 话题设置 ---
                'aoa_topic': '/aoa_angle',
                'snr_topic': '/snr_db',
                'marker_topic': '/aoa_markers',

                # --- TF / 几何门限 & TX 位置 ---
                'map_frame': 'map',
                'base_frame': 'base_link',
                'tx_x': 0.129925,
                'tx_y': 1.860273,
                'aoa_abs_deg_limit': 32.0,
                'angle_in_degrees': False,

                # --- AOA 箭头外观 ---
                'arrow_length': 1.0,
                'shaft_diameter': 0.03,
                'head_diameter': 0.08,
                'head_length': 0.15,

                # --- 航向（蓝色）箭头 ---
                'orient_arrow_enable': True,
                'orient_arrow_length': 1.2,
                'orient_shaft_diameter': 0.03,
                'orient_head_diameter': 0.08,
                'orient_head_length': 0.15,

                # --- 轨迹采样与渐隐（紫罗兰色） ---
                'trail_period': 6.0,               # 每 6 秒采样一条 AoA 线段
                'trail_length_min': 3.0,            # SNR_min → 3 m
                'trail_length': 10.0,               # SNR_max → 10 m（最大长度）
                'trail_line_width': 0.02,
                'trail_alpha_max': 0.9,
                'trail_fade_half_life_sec': 20.0,
                'trail_max_age_sec': 60.0,
                'trail_capacity': 10,

                # --- SNR → 长度映射 ---
                'snr_min_db': -30.0,                  # 最短线对应的 SNR
                'snr_max_db': 0.0,                 # 最长线对应的 SNR

                # --- SNR 异常检测（可按需开启/关闭） ---
                'snr_outlier_enable': True,
                'snr_window_sec': 5.0,
                'snr_min_samples': 5,
                'snr_drop_db': 15.0,
                'snr_k_mad': 2.5,
                'snr_rel_drop_frac': 0.6,
                'snr_hold_sec': 0.5,
            }]
        ),
    ])
