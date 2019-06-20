# Yumi Manipulation of Shoe Laces - AI Planning and Control

## Dependencies
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [OPENNI](http://wiki.ros.org/openni_camera) for ASUS or Kinect
- [ZED ROS](https://github.com/stereolabs/zed-ros-wrapper) wrapper for Zed mini
- [Python PCL](https://medium.com/@ss4365gg/%E6%88%90%E5%8A%9F%E5%9C%A8ubuntu-16-04%E7%92%B0%E5%A2%83%E4%B8%AD%E5%AE%89%E8%A3%9D-pcl-1-8-1-python-pcl-a016b711bc4) which depends on PCL
- [YOLO ROS](https://github.com/leggedrobotics/darknet_ros) for real-time object detection
- The [YuMi ROS](https://github.com/ImperialCollegeLondon/yumi-prl) wrapper and any further dependencies described there
- [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

## Files
1. The pose relationships between the two cameras and YuMi are defined in the YuMi launch files. The **demo.launch** is for simulation in Rviz, **yumi.launch** is for launching the real robot. 

2. The resolution and other settings of ZED Mini camera can be modified in the **common.yaml** file in the ```zed-ros-wrapper``` package. Current settings can be found in the ```file``` folder.

3. To use different versions of YOLO, adding corresponding **.cfg** and **.weights** files to ```yolo_network_config``` folder, and points them in the **yolo.yaml** file in the ```/darknet_ros package/config``` folder. YOLO9000 related files are can be found in the ```file``` folder.

4. Folder ```shoehole```: computer vision related files

- **shoe_asus.py**: provides functions of shoe detection, calculation of required locations for shoe pose adjustment, 6D shoe hole pose estimation etc using ASUS Xtion camera.
- **shoe_zed.py**: provides same functionalities as shoe_asus.py except it is for ZED Mini camera.

5. Folder ```shoelace```: motion planning related files

- **go_asus.py**: provides shoe pose adjustment, shoelace insertion, grabbing, and pulling, as well as offset adjustment functionalities while using ASUS Xtion camera.
- **go_zed.py**: provides same function as go\_asus.py except it is for ZED Mini camera.
- **yumi_moveit_utils.py**: includes several functions which are used in go_asus.py and go_zed.py.

## Usage

### YuMi setup
To launch this project, YuMi needs to be set up firstly. To do this, please follow these steps:

1. Turn on YuMi through the power switch.

2. Connect the Ethernet cable on XP23 port to your machine.

3. Turn on motors on controller interface (toggle physical button with 3 horizontal lines on the FlexPendant).

4. Switch to auto mode (toggle physical button with 2 horizontal lines on the FlexPendant).

5. Point to main programs (toggle physical button with 1 horizontal line on the FlexPendant).

6. Press the play button, both YuMi's grippers should then calibrate.

### Quick start: shoe and shoelace manipulation
Once YuMi has been set up, the shoe can be placed on the workbench. When using ZED Mini camera, following command should be run from a terminal after ```roscore```:

1. ``` roslaunch zed_wrapper zedm.launch ``` to launch ZED Mini camera

2. ``` roslaunch yumi_shoe yumi.launch ``` to launch YuMi ROS Nodes

3. ``` roslaunch darknet_ros darknet_ros.launch ``` to launch YOLO detection

4. ``` rosrun yumi_shoe shoe_zed.py ``` to launch computer vision module

5. ``` rosrun yumi_shoe go_zed.py ``` to launch motion planning module

### Run YuMi simulation

``` roslaunch yumi_shoe demo.launch ```

### While using ASUS Xtion camera

``` roslaunch openni2_launch openni2.launch ``` to launch ASUS Xtion camera

``` rosrun yumi_shoe shoe_asus.py ``` to launch computer vision module

``` rosrun yumi_shoe go_asus.py ``` to launch motion planning module

