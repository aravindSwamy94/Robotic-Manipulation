# lumi_testbed

The set of core ROS packages for lumi robot. Contains URDF description,  moveit configuration, mujoco configuration.

## Installation
Prerequisites:
```sh
sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros
```
Workspace creation:
```sh
mkdir -p ~/ros/src
cd ~/ros/src
git clone git@version.aalto.fi:robotic_manipulation/mujoco_ros_control.git
git clone git@version.aalto.fi:lumi/lumi_testbed.git
cd ~/ros
colcon build
```
## Launch robot
Simulated robot without MuJoCo and MoveIt:
```sh
roslaunch lumi_description show.launch 
```

Simulated robot with MuJoCo and MoveIt:
```sh
roslaunch lumi_mujoco simulation.launch
```

Real robot:
```sh
TBD. 
```
