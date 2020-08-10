# Hand Eye Calibration for TM700 robot

If calibration is done, just launch the publish launch file.
## Setup

### Python
Calibration depends upon `pyrobot`, `numpy`, and `flask` (web-based gui)

### ROS
The `ros-tm700` and `easy_handeye` are in the submodules

```
git submodule init
git submodule update
```

Also need:
1. A standard ros camera. We use [realsense](https://github.com/IntelRealSense/realsense-ros) here.
2. [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) (Check the docs and print AR tag)


## Run calibration (one time)

### Launch camera (optional)
```
roslaunch realsense2_camera rs_rgbd.launch  color_width:="1280" color_height:="720" depth_width:="1280" depth_height:="720" infra_width:="1280" infra_height:="720" color_fps:="6" depth_fps:="6" infra_fps:="6"
```
### Launch main calibration
```
roslaunch calibration calibration.launch robot_ip:=[ROBOT IP]
```

## Publish Estimated Transforms (Launch everytime)
The launch file **includes** all the tm700 main ros packages and gripper.
```
roslaunch calibration publish.launch robot_ip:=[ROBOT IP]
```
