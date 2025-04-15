
- cd ros2_ws
    `colcon build --symlink-install # build workspace for dev`

- install the build
    `source ./install/setup.bash`

- launch 
    `ros2 launch arena_camera_node arena_camera.launch.py`

- params are in ./src/arena_camera_node/launch/arena_camera.launch.py
