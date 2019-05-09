# FYP-Yumi

install openni

install imperial college yumi ...

install pcl, python-pcl https://medium.com/@ss4365gg/%E6%88%90%E5%8A%9F%E5%9C%A8ubuntu-16-04%E7%92%B0%E5%A2%83%E4%B8%AD%E5%AE%89%E8%A3%9D-pcl-1-8-1-python-pcl-a016b711bc4

```
roslaunch openni_launch openni.launch device_id:=#2
rosrun shoe_detection shoe_position.py 
roslaunch yumi_moveit_config yumi.launch 
```

# motion_intent

ROS package for interfacing with YuMi and communicating intended trajectories via the HoloLens. See the [HoloYuMi](https://github.com/ImperialCollegeLondon/HoloYuMi) repository for the Unity project corresponding to the HoloLens app that works alongside this ROS package.

## Dependencies

The following packages must be installed in your workspace:
- [YOLO ROS](https://github.com/leggedrobotics/darknet_ros) for real-time object detection
- The [file_server](https://github.com/siemens/ros-sharp/tree/master/ROS/file_server) package for ROS-HoloLens message passing via [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- The [YuMi ROS](https://github.com/ImperialCollegeLondon/yumi-prl) wrapper and any further dependencies described there

## Launch

First you need to setup YuMi. To do this, follow these steps:

1. Turn on YuMi through the power switch
2. Connect the ethernet cable to your machine
3. Turn on motors on controller interface (three horizontal lines button)
4. Switch to auto mode (two horizontal lines button)
5. Point to main programs (one horizontal line button)
6. Press the play button, grippers should calibrate
7. Run the following command from a terminal (after `roscore`):
```shell
roslaunch motion_intent yumi.launch
```

Once YuMi is purring happily with an RViz view of the motion planning, you can then launch the object detection from a separate terminal:
```shell
roslaunch motion_intent object_recognition.launch
```
Another window should appear with the objects detected by the camera placed on top of YuMi's base. If not, the camera has possibly not been set up correctly (requires a calibration `.yaml` file, typically saved in the hidden `~/.ros/camera_info` directory).

The next step is to setup the ROS bridge:
```shell
roslaunch file_server ros_sharp_communication.launch
```
With all the above processes running, you can now switch over to the HoloLens and open the "HoloYuMi" app. If the networks are correctly configured, the ROS bridge terminal should tell you that it has found a client.

Finally, run the motion intent node to interact with the shared workspace using YuMi from the HoloLens:
```shell
rosrun motion_intent motion_intent_node cmd:=/yumi/command bounding_boxes:=/darknet_ros/bounding_boxes point_cloud:=/camera/depth_registered/points
```

You should now be able to command YuMi to interact with the cups and teddies on the table by tapping interface buttons rendered in the HoloLens view.

Talk to Mark if you're having troubles with setting up.
