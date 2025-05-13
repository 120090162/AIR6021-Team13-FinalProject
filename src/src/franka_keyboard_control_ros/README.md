```bash
colcon build --packages-up-to franka_keyboard_control_ros
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.zsh
ros2 launch franka_keyboard_control_ros franka_interface.launch.py
ros2 run franka_keyboard_control_ros rm_servo_keyboard_input
```

```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```