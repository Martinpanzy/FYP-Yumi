# FYP-Yumi

## Dependencies
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [OPENNI](http://wiki.ros.org/openni_camera) for ASUS or Kinect
- [ZED ROS](https://github.com/stereolabs/zed-ros-wrapper) wrapper for Zed mini
- [Python PCL](https://medium.com/@ss4365gg/%E6%88%90%E5%8A%9F%E5%9C%A8ubuntu-16-04%E7%92%B0%E5%A2%83%E4%B8%AD%E5%AE%89%E8%A3%9D-pcl-1-8-1-python-pcl-a016b711bc4) which depends on PCL
- [YOLO ROS](https://github.com/leggedrobotics/darknet_ros) for real-time object detection
- The [YuMi ROS](https://github.com/ImperialCollegeLondon/yumi-prl) wrapper and any further dependencies described there
- [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

## Run shoe lacing
```
roscore
roslaunch zed_wrapper zedm.launch --or-- roslaunch openni2_launch openni2.launch
roslaunch yumi_shoe yumi.launch 
roslaunch darknet_ros darknet_ros.launch 
rosrun yumi_shoe shoe_zed.py --or-- rosrun yumi_shoe shoe_asus.py
rosrun yumi_shoe go_zed.py --or-- rosrun yumi_shoe go_asus.py 
```

## Run YuMi simulation
```
roslaunch yumi_shoe demo.launch 
```
