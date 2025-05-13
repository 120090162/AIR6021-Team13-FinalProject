```bash
colcon build --packages-up-to franka_keyboard_control
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.zsh
ros2 launch franka_keyboard_control franka_interface.launch.py
ros2 run franka_keyboard_control rm_servo_keyboard_input
ros2 launch franka_keyboard_control franka_control.launch.py

ros2 run joint_state_pkg joint_state_subscriber

# 一键启动
ros2 launch franka_keyboard_control bringup.launch.py
```

```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```

```bash
./compile /home/joshua/WORK/TEMP/robot_arm_simulation/urdf/robot_arm_simulation.urdf /home/joshua/WORK/TEMP/ws_moveit/src/robot_arm_simulation/xml/robot_arm_simulation.xml

# test
./simulate /home/joshua/WORK/TEMP/ws_moveit/src/robot_arm_simulation/xml/robot_arm_simulation.xml
```