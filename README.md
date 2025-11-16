# ROS2 Programming Assignments 
- Assignment 1 - Publisher / Subscriber
- Assignment 2 - Logging and Services
- Assignment 3 - Tf2, unit testing, and bag files

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

ros2 launch beginner_tutorials launch_nodes.py publish_rate:=1.0 # publish_rate is a modifiable parameter
```

### How to call service
```bash
ros2 service call /modify_message beginner_tutorials/srv/ModifyMessage "{modified_message: '<write message here>'}"
```

Now you should see a message being published to the terminal from the Talker node, and the Listener node recieving and publishing the same message in the other terminal.

### How to view TF frames
```bash
# To echo the tf frames that were published run
ros2 run tf2_ros tf2_echo ...

# To view the transform frames run
ros2 run tf2_tools view_frames

# This will save a pdf of the frames and their parents currently being published. File is saved in the directory the command was run.
```

### How to run catch2 integration tests
```bash
# Make sure catch2 is installed on your machine. Refer to depencies section at the bottom for install steps
# Launch the integration tests
ros2 launch beginner_tutorials integration_test.launch.yaml

# The integration test will be run and the results will print to the terminal. For an example of the results view catch2_integration_tests.png in results directory
```
### How to record a ros2 bag
```bash
# Make sure both ros2 underlay and overlay are soruced properly, then run the following command:
ros2 launch beginner_tutorials rosbag_record_topics.launch.py enable_bag_recording:=true

# To launch the nodes without recording the bag file run
ros2 launch beginner_tutorials rosbag_record_topics.launch.py enable_bag_recording:=false
```

### Inspecting and playing bag back
```bash
# Inspect the bag's content with 
ros2 bag info <bag name>

# Play back the bag's contents with
ros2 bag play <bag name>
```

### Ros2 bag replay demo
```bash
# Run the listener node
ros2 run beginner_tutorials listener

# In a separate terminal, play the bag contents
ros2 bag play <bag name>
```
Change back to the terminal the listener is running in. You should see the listener recieving messages from the bag.


## Dependencies
This package assumes that you have ROS2 Humble properly installed and setup on your machine. If this is not the case, the steps to install ROS2 can be found [here](https://docs.ros.org/en/humble/Installation.html).

The integration tests for this package depend on catch2. To install catch2 run the following:
```bash
source /opt/ros/humble/setup.bash  # if needed
apt install ros-${ROS_DISTRO}-catch-ros2
```