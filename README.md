
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

## Prepare object_detection
```
$ cd demo/src/travel/
$ git clone https://github.com/amslabtech/object_detection.git
$ cd object_detection/
$ ln -s keras-yolo3/yolo3 .
$ wget https://pjreddie.com/media/files/yolov3.weights
$ python3 keras-yolo3/convert.py yolov3.cfg yolov3.weights model_data/yolo3/coco/yolo.h5

$ git clone https://github.com/matterport/Mask_RCNN.git
$ ln -s Mask_RCNN/mrcnn .
$ pip3 install -r Mask_RCNN/requirements.txt
$ wget https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5 -P model_data/mrcnn/coco/

```

## Gazebo
```
$ sudo apt install ros-crystal-gazebo-*
$ cd gazebo
$ source /opt/ros/crystal/setup.bash
$ source /usr/share/gazebo/setup.sh
$ gazebo --verbose demo_world56.world
$ killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
```

## Run
```
$ cd demo
$ source /opt/ros/crystal/setup.bash
$ colcon build
$ source install/setup.bash && source install/local_setup.bash
$ ros2 run travel image_publisher
$ ros2 run travel object_detection_publisher
$ ros2 run travel agent
```