# FYP-Yumi

install openni

install imperial college yumi ...

install pcl, python-pcl https://medium.com/@ss4365gg/%E6%88%90%E5%8A%9F%E5%9C%A8ubuntu-16-04%E7%92%B0%E5%A2%83%E4%B8%AD%E5%AE%89%E8%A3%9D-pcl-1-8-1-python-pcl-a016b711bc4

install darknet_ros

```
roscore
roslaunch openni2_launch openni2.launch
roslaunch yumi_moveit_config yumi.launch 
roslaunch darknet_ros darknet_ros.launch 
rosrun shoe_detection shoe_position.py
rosrun yumi_moveit_demos main.py 
```
