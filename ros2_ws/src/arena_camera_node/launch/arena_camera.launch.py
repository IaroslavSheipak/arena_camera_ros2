from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_camera_node',
            executable='start',  # Ensure this matches your actual executable name
            name='arena_camera',
            parameters=[
                {
                    "device_link_throughput_limit": 125000000,
                    "gev_scpd": 0,
                    'pixelformat': 'rgb8',
                    'width': 1440,
                    'height': 1080,
                    'gain': 20.0,           # higher gain to brighten
                    # 'exposure_time': 10000.0,  # 10 ms in microseconds
                    'acquisition_frame_rate': 25.0,
                    'trigger_mode': False,
                    'qos_history': 'keep_last',
                    'qos_history_depth': 10,
                    'qos_reliability': 'reliable'
                }
            ]
        )
    ])
