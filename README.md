# DYROS Robot Menagerie
---
## Dependencies

- [MuJoCo ROS Sim](https://github.com/JunHeonYoon/mujoco_ros_sim.git)
- [DYROS Robot Controller](https://github.com/JunHeonYoon/dyros_robot_controller.git)  
---

## Installation
Install step and running code example
```bash
cd ~/ros2_ws
git clone https://github.com/JunHeonYoon/dyros_robot_menagerie.git src
colcon build --symlink-install
source install/setup.bash

ros2 launch dyros_robot_menagerie fr3.launch.py # husky, xls, pcv, ur5e 
```
