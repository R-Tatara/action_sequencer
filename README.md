# Action Sequencer
[![Linux](https://custom-icon-badges.herokuapp.com/badge/Linux-F6CE18.svg?logo=Linux&logoColor=white)]()
[![ROS2](https://img.shields.io/badge/-ROS2%20Humble-22314E.svg?logo=ros&style=flat)]()
[![C++](https://custom-icon-badges.herokuapp.com/badge/C++-00599C.svg?logo=cplusplus&logoColor=white)]()
<img src="https://img.shields.io/badge/-Ubuntu%2022.04-6F52B5.svg?logo=ubuntu&style=flat">

A ROS2 / MoveIt2 sample node that executes a predefined pick-and-place motion sequence with the Motion Planner.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2

## Installation

Clone MELFA ROS2 Driver.

See [melfa_ros2_driver](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_driver) GitHub repository.

```bash
# Clone into your colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/R-Tatara/action_sequencer.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Usage

```bash
# 1. Launch your robot + MoveIt
[Terminal 1]
ros2 launch melfa_bringup rv7frl_control.launch.py use_fake_hardware:=true controller_type:="R"
[Terminal 2]
ros2 launch melfa_rv7frl_moveit_config rv7frl_moveit.launch.py

# 2. Run the action sequencer
[Terminal 3]
ros2 launch action_sequencer action_sequencer.launch.py arm_model:=rv7frl
```

### What It Does

1. Initializes MoveIt `MoveGroupInterface` with the Pilz PTP planner
2. Adds a collision box to the planning scene
3. Executes the motion sequence:

## License

This project is licensed under the [MIT License](LICENSE).