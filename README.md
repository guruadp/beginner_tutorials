# beginner_tutorials

## Overview

This projects contains a publisher and subscriber node that publishes custom string message to a topic and the subscriber listens to the string published by the publisher node from the same topic.

## Build/run steps

Create a workspace 

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
Clone the repo into src folder
```
git clone https://github.com/guruadp/beginner_tutorials.git
```
Build the package
```
colcon build --packages-select cpp_pubsub
```
Source the terminal and run publisher

```
. install/setup.bash
ros2 run cpp_pubsub talker
```

Open a new terminal, source it and run subscriber

```
. install/setup.bash
ros2 run cpp_pubsub listener
```


## Dependencies

ROS humble - To install follow the below link

http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
