# FYP-Yumi

## Dependencies
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [openni](http://wiki.ros.org/openni_camera) for ASUS or Kinect
- [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper) for Zed mini
- [python-pcl](https://medium.com/@ss4365gg/%E6%88%90%E5%8A%9F%E5%9C%A8ubuntu-16-04%E7%92%B0%E5%A2%83%E4%B8%AD%E5%AE%89%E8%A3%9D-pcl-1-8-1-python-pcl-a016b711bc4) which depends on pcl
- [YOLO ROS](https://github.com/leggedrobotics/darknet_ros) for real-time object detection
- The [YuMi ROS](https://github.com/ImperialCollegeLondon/yumi-prl) wrapper and any further dependencies described there
- [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

## Run shoe lacing
```
roscore
roslaunch zed_wrapper zedm.launch --or-- roslaunch openni2_launch openni2.launch
roslaunch yumi_moveit_config yumi.launch 
roslaunch darknet_ros darknet_ros.launch 
rosrun shoehole shoe_zed.py --or-- rosrun shoehole shoe_asus.py
rosrun shoelace go_zed.py --or-- rosrun shoelace go_asus.py 
```

## Run YuMi simulation
```
roslaunch yumi_moveit_config demo.launch 
```
