# ROS2 Programming Assignment 1 - Publisher / Subscriber

## Overview
This repository is a simple ROS2 package that follows the **Writing a simple publisher and subscriber (C++)** tutorial from the [ROS2 Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). The package creates 2 nodes, 1 talker node that publishes a message to a topic and displays the message in the terminal, and 1 listener node that subscribes to the same topic and displays the recieved message from the talker node.

## How to Build/Run

### Create ROS2 Workspace
```bash
# Source ROS2 underlay
source /opt/ros/humble/setup.bash

# Navigate to the driectory location where you wish to create a ROS2 workspace
mkdir ros2_ws
cd ros2_ws/
mkdir src
cd src/
cd ..

colcon build
```
### Clone and Build Repository
```bash
cd src/
git clone https://github.com/GraysonGilbert/my_beginner_tutorials.git
cd ..
colcon build --packages-select beginner_tutorials

```

### Run Listener Executable
```bash
# From the same terminal that you just built the package in run the following:
. install/setup.bash
ros2 run  beginner_tutorials listener
```

### Run Talker Executable

Open a new terminal and navigate to the ROS2 workspace directory previously created.

```bash
# Source ROS2 underlay
source /opt/ros/humble/setup.bash
# Source overlay
. install/setup.bash

ros2 run  beginner_tutorials talker
```

### Launch both nodes at the same time with launch file
```bash
# Source ROS2 underlay
source /opt/ros/humble/setup.bash
# Source overlay
. install/setup.bash

ros2 launch beginner_tutorials launch_nodes.py publish_rate:=1.0
```

### How to call service
```bash
ros2 service call /modify_message beginner_tutorials/srv/ModifyMessage "{modified_message: '<write message here>'}"
```

Now you should see a message being published to the terminal from the Talker node, and the Listener node recieving and publishing the same message in the other terminal.

## Dependencies
This package assumes that you have ROS2 Humble properly installed and setup on your machine. If this is not the case, the steps to install ROS2 can be found [here](https://docs.ros.org/en/humble/Installation.html).