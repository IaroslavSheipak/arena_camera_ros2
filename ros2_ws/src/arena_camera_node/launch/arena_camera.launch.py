# arena_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_camera_node',
            executable='start',          # use your real executable name
            name='arena_camera',
            parameters=[{
                # ── Ethernet tuning ─────────────────────────────
                "device_link_throughput_limit": 125_000_000,      # 1 Gb/s
                # "gev_scpd": 0,                                 # inter-packet delay (optional)

                # ── Image format & ROI ─────────────────────────
                "pixelformat": "rgb8",
                "width":       1440,
                "height":      1080,

                # ── Automatic controls (plain GenICam) ────────
                "genicam.TargetBrightness": 70,                  # AE target
                "genicam.GammaEnable":      True,
                "genicam.Gamma":            0.5,
                "gain_auto":  "Continuous",                      # override manual gain
                "balance_white_auto": "Continuous",
                # "genicam.ExposureAuto": "Continuous",          # default already

                # ── Frame rate ─────────────────────────────────
                "acquisition_frame_rate": 10.0,                  # FPS cap

                # ── Trigger mode off (streaming) ──────────────
                "trigger_mode": False,

                # ── QoS for the /images topic ─────────────────
                "qos_history":        "keep_last",
                "qos_history_depth":  10,
                "qos_reliability":    "reliable"
            }]
        )
    ])
