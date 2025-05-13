from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #
    # ⚙️  Parameters for the refactored driver
    #
    params = {
        # ---- generic camera settings ------------------------------------
        "pixelformat": "rgb8",
        "width": 1440,
        "height": 1080,
        "acquisition_frame_rate": 10.0,     # FPS
        "trigger_mode": False,              # stream continuously

        # ---- GenICam overrides (anything after the dot is written 1-for-1)
        #      They match the defaults we discussed for CRC night-to-day use.
        "genicam_overrides.AutoFunctionsTargetGreyValue": 90,
        "genicam_overrides.AutoGainUpperLimit": 6,            # dB
        "genicam_overrides.AutoExposureTimeUpperLimit": 90000,  # µs  (0.09 s @10 fps)

        # Ethernet bandwidth cap (Camera → NIC)
        "genicam_overrides.DeviceLinkThroughputLimitMode": "On",
        "genicam_overrides.DeviceLinkThroughputLimit": 125000000,  # ≈1 Gb s-¹

        # If you *really* want continuous auto-white-balance, uncomment next line
        # "genicam_overrides.BalanceWhiteAuto": "Continuous",
    }

    #
    # NOTE ─────────────
    # • `package` must match your ROS 2 package name.
    # • `executable` must match the target you install:
    #     - If you built the node as a stand-alone exe → use that filename
    #       (e.g. `arena_camera_node_refactored`).
    #     - If you registered it only as a component you’d instead launch it
    #       via a ComposableNodeContainer.
    #
    return LaunchDescription([
        Node(
            package="arena_camera_node",
            executable="start",
            name="arena_camera",
            parameters=[params],
        )
    ])
