# liske_tag_publisher

ROSNode that takes in mpa data and published it to ROS. There are 9 pieces of metadata with each tag detection publish.

## Simple Installation of Prebuilt liske_tag_publisher_0.0.5 ipk

0. Download and Install Prerequisites
    * Make sure adb is set up correctly on voxl and PC: https://docs.modalai.com/setting-up-adb/
    * Download the liske_tag_publisher repository

1. Install Pre-Built ipk Package onto Voxl
    * on (PC), run the command``` cd [Path To]/liske_tag_publisher```
    * followed by (PC) ```./deploy_to_voxl.sh```



## Complete Build Instructions and Installation

0. Download and Install Prerequisites
    * Download the liske_tag_publisher repository
    * Make sure adb is set up correctly on voxl and PC: https://docs.modalai.com/setting-up-adb/
    * Make sure docker is set up correctly on host PC: https://docs.modalai.com/voxl-docker-and-cross-installation/

1. Install and run voxl-emulator docker image
    * If voxl-emulator is not already installed:
    * Download the voxl-docker Repository (found [here](https://gitlab.com/voxl-public/support/voxl-docker))
    * Download the prebuilt voxl-emulator image found ([here](https://developer.modalai.com/asset/eula-download/75)) and place it inside the voxl-docker directory
    * (PC) ```cd [Path To]/voxl-docker```
    * (PC) ```sudo sh install-emulator-docker.sh``` 
    * (PC) Verify installation with ```docker images``` and make sure that voxl-emulator is present on the list
    * If voxl-emulator is installed continue here:
    * (PC) ```cd [Path To]/liske_tag_publisher```
    * (PC) ```sudo voxl-docker -i voxl-emulator:V1.7```  (or whatever version you are using)
2. Build project binary:
    * (VOXL-EMULATOR) ```./install_build_deps.sh apq8096 dev``` 
    * (VOXL-EMULATOR) ```./clean.sh```
    * (VOXL-EMULATOR) ```./build.sh apq8096```
    * (VOXL-EMULATOR) ```./make_package.sh ipk```
3. Install built ipk package on Voxl:
    * (PC) ```./deploy_to_voxl.sh```

 
### Start Installed liske_tag_publisher Node

Connect to the voxl through adb or ssh:

(PC) ```adb shell``` or ```sudo ssh root@192.168.8.101``` or whatever the voxl's ip address is



Run the following commands(on voxl):

Verify the configuration your ros environment with:
```
vi ~/my_ros_env.sh
```

if you make any changes make sure to run ```exec bash``` to re-source the file

start a ros master on the voxl with
```
roscore
```

and then, in a new terminal, ```adb shell``` or ```sudo ssh``` into voxl, and start the node with:

```
rosrun liske_tag_publisher liske_tag_publisher_node
```
## AprilTag Setup

All tags that are to be used MUST be properly enabled in the config file on the voxl, or the location xyz vector and rotation matrix will be filled with NaN. 

The file location is ```/etc/modalai/tag_locations.conf```

It is important to configure the size of each tag, so that the xyz tag detection distances are correct.

More information about the configuration file found [here](https://docs.modalai.com/voxl-tag-detector-0_9/#tag-detector-configuration-file)

## Published Data Information

The publisher publishes the MPA data taken from the MPA channel "tag_detections" that is output from the voxl-tag-detector service. It has the following fields
    
    * int32 id                           #id number of detected tag
    * float32 size_m                     #size of tag in meters
    * int64 timestamp_ns                 #timestamp at middle of the frame exposure in monotonic time
    * int32 loc_type                     #location type
    * float64[] T_tag_wrt_cam            #vector of length 3, representing tag position [x,y,z] relative to camera frame 
    * float64[] R_tag_to_cam             #vector of length 9, flattened 3x3 rotation matrix, relative to camera frame, rotation order yaw-pitch-roll
    * float64[] T_tag_wrt_fixed          #vector of length 3, representing tag position [x,y,z] relative to fixed frame
    * float64[] R_tag_to_fixed           #vector of length 9, flattened 3x3 rotation matrix, relative to fixed frame, rotation order yaw-pitch-roll
    * float64[] RPY_to_cam               #vector of length 3, containing the tag's rotation about its X/Y/Z axes with respect to the camera frame, in degrees
    * int32 reserved                     #reserved
    
## Troubleshooting

1. Make sure that the voxl-tag-detector service is running. Check ```voxl-inspect-services``` to verify. If its not running use ```voxl-configure-tag-detector``` to restart the service

2. Not launching? Make sure that roscore is running on a terminal on the drone before launching the node with rosrun

3. It's not doing anything? no text is published to the terminal. Check rostopic list to verify that /tag_detection_t topic is visible, then use ```rostopic echo /tag_detection_t``` to see the data being published to the topic with each message

## Additional Documentation

voxl-tag-detector documentation https://docs.modalai.com/voxl-tag-detector-0_9

<!-- ### Expected Behavior
```
voxl:/$ roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
... logging to /home/root/.ros/log/5ef19600-cbd7-11ec-8a51-ec5c68cd23bd/roslaunch-apq8096-3468.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.58:46256/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /
    voxl_mpa_to_ros_node (voxl_mpa_to_ros/voxl_mpa_to_ros_node)

auto-starting new master
process[master]: started with pid [3487]
ROS_MASTER_URI=http://192.168.1.58:11311/

setting /run_id to 5ef19600-cbd7-11ec-8a51-ec5c68cd23bd
process[rosout-1]: started with pid [3500]
started core service [/rosout]
process[voxl_mpa_to_ros_node-2]: started with pid [3503]


MPA to ROS app is now running

Found new interface: stereo
Found new interface: tof_conf
Found new interface: tof_depth
Found new interface: tof_ir
Found new interface: tof_noise
Found new interface: tracking
Found new interface: tof_pc

``` -->
