# wut_velma_gazebo

## Contents


## Dependencies

The Package was tested on the following setup:
* Ubuntu 24.04
* ROS 2 Jazzy
* Python 3.12


## Installation

```bash
clear && python3 -m colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Run

Run Simulation:
```bash
cd ~/ws_wut_velma
source install/setup.bash
ros2 launch wut_velma_gazebo empty_world.launch.py
```
