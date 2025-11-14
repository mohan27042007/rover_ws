# rover_ws
Phase-01 Completed

#Follow these Steps for ready deployment:
🚀 Rover Simulation — Phase 1

## Requirements

-Ubuntu 24.04
-ROS 2 Jazzy Jalisco
-Gazebo Harmonic

## Installed packages:

Make sure to install the following ROS 2 Jazzy Jalisco packages:

```bash
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    libgz-sim8-dev \
    libgz-msgs11-dev \
    libgz-transport13-dev \
    libgz-math8-dev \
    libgz-common5-dev \
    gz-harmonic

```
### Clone the Repository

Clone this repository into your ``workspace/src`` folder. If you don't have a workspace set up, you can learn more about creating one in the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).


```bash
cd <path_to_your_workspace>/src
git clone <https link>
cd ..
```
or Use VS Code Editor's ``Command pallet`` and select ``Git Clone`` command, and Copy-Paste the https link of the repo in the command.

### Build the colcon package and Launch:
```bash
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_gz_sim robot.launch.py
```

### Teleoperation Command:
On a different Terminal: 

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rover_bringup teleop.launch.py
```
