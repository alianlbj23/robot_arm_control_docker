colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install 
source install/setup.bash
# ros2 run pros_arm_py keyboard
# ros2 run pros_arm_py random
# ros2 run pros_arm_py mock
# ros2 run pros_arm_py arm_reader
# ros2 run pros_arm_py arm_writer
# ros2 run pros_arm random_target
ros2 launch pros_arm_py arm_reader_and_writer.launch.py
