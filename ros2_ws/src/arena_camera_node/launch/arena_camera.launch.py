from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_camera_node',
            executable='start',          # keep this in sync with CMake
            name='arena_camera',
            parameters=[{
                # ── transport & networking ───────────────────────────────
                "device_link_throughput_limit": 125_000_000,   # 125 MB/s
                
                # ── image format & geometry ──────────────────────────────
                "pixelformat": "rgb8",      # or "mono8" if you switch later
                "width":        1440,
                "height":       1080,

                # ── auto-exposure / gain strategy ───────────────────────
                "gain_auto":                 "Continuous",
                "exposure_auto":             "Continuous",  # default, but set explicitly if your node supports it
                "exposure_auto_damping":      69.0,    # %   (0–100)    -- smooth but responsive

                "target_brightness":          70.0,   # 0–255 mid-grey target
                "gamma":                        0.5,
                "balance_white_auto":        "Continuous",

                # ── acquisition timing ─────────────────────────────────
                # "acquisition_frame_rate":      10.0,   # Hz
                "trigger_mode":               False,   # free-run

                # ── ROS2 QoS settings ──────────────────────────────────
                "qos_history":        "keep_last",
                "qos_history_depth":  10,
                "qos_reliability":    "reliable"
            }]
        )
    ])
