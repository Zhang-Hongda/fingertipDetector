fingertip_detector
================================
****
|Author|Zhang Hongda|
|------|-----------------|
|E-mail|2210010444@qq.com|
|Institute|Harbin Institute of Technology|
****

### Description
A ROS node for tracking the position of a fingertip in camera frame.
### Installation
Step 1: clone the repository into your own workspace
```
cd ${PATH_TO YOUR_WORKSPACE_FOLDER}/src
git clone https://github.com/Zhang-Hongda/fingertip_detector
```
Step 2: building
```
catkin_make
```
Step 3: activate the workspace
```
source ${PATH_TO YOUR_WORKSPACE_FOLDER}/devel/setup.bash
```
### Strat 
To start the program, run:
```
roslaunch fingertip_detector fingertip_detector.launch 
```
### Application
* The package can be integrated with [pcl_tracker ](https://github.com/Zhang-Hongda/pcl_tracker) and [move_ur5_qt](https://github.com/Zhang-Hongda/move_ur5_qt) to form a system of robot trajectories programming using a fingertip.
* After successfully built all those three packages, run:
```
roslaunch fingertip_detector fingertip-RPD.launch 
```

