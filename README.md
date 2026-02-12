# wut_velma_gazebo

## Contents


## Dependencies

The Package was tested on the following setup:
* Ubuntu 24.04
* ROS 2 Jazzy
* Python 3.12


## Installation


## Run

Run Simulation:
```bash
cd ~/ws_wut_velma
source install/setup.bash
ros2 launch wut_velma_gazebo empty_world.launch.py
```

Convert urdf xacro of the robot to Gazebo model (sdf):
```bash
ros2 launch wut_velma_gazebo generate_velma_sdf.launch.py
```

