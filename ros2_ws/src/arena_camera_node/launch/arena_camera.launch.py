from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cam_params = {
        # ── basic image settings ─────────────────────────────────────────────
        "pixelformat": "rgb8",
        "width": 1440,
        "height": 1080,
        "acquisition_frame_rate": 10.0,     # FPS   (leave <0 for camera default)
        "trigger_mode": False,              # True → wait for /trigger_image

        # ── ROS topic / QoS ──────────────────────────────────────────────────
        "topic": "/cam/image_raw",          # where publishers push frames
        "qos_reliability": "reliable",      # reliable | best_effort
        "qos_history": "keep_last",         # keep_last | keep_all
        "qos_history_depth": 10,            # only when keep_last

        # ── Ethernet traffic knobs ───────────────────────────────────────────
        # uncapped by default; set bits/s to throttle
        "device_link_throughput_limit": 125_000_000,  # 1 Gb/s
        # inter-packet delay (ticks, often 1 µs); -1 means “leave as is”
        # "gev_scpd": 1000,

        # ── open-ended GenICam overrides (prefix = genicam_overrides.) ───────
        "genicam_overrides.AutoFunctionsTargetGreyValue": 90,
        "genicam_overrides.AutoGainUpperLimit": 6,          # dB
        "genicam_overrides.AutoExposureTimeUpperLimit": 90000,  # µs
        # "genicam_overrides.BalanceWhiteAuto": "Continuous",
    }

    # NOTE:
    # • package= must match your package (default from examples: arena_camera_node)
    # • executable= is the installed binary name, e.g. `arena_camera_node_refactored`
    #
    return LaunchDescription([
        Node(
            package="arena_camera_node",
            executable="start",  # <-- change if different
            name="arena_camera",
            output="screen",
            parameters=[cam_params],
        )
    ])
