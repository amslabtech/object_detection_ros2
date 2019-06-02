
## Install Ros2
```

$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ curl http://repo.ros2.org/repos.key | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ export CHOOSE_ROS_DISTRO=crystal
$ sudo apt update
$ sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop                     # $CHOOSE_ROS_DISTRO= crystal
$ sudo apt install python3-argcomplete
$ sudo apt install python3-colcon-common-extensions
```

Prepare object_detection
```
cd demo/src/travel/
https://github.com/amslabtech/object_detection.git
Prepare package and model by following README in object_detection
```

## Run
```
$ cd demo
$ source /opt/ros/crystal/setup.bash
$ colcon build
$ source install/setup.bash && source install/local_setup.bash
$ ros2 run travel demo_yolo
```