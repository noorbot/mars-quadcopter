# MarsQuadcopter
Development space for the MARS Lab Quadcopter research project at Ontario Tech University, by Noor Khabbaz. All work was done in ROS1 Noetic.

## Background
This project had two main goals when it comes to UAV-UGV collaboration:
1. **Robot localization:** Enable UAV to estimate UGV relative poses via their ArUco Markers.
2. **Obstalce mapping:** Enable UAV to map obstacles using a pointcloud method to be shared with UGVs for avoidance.

### Robot Localization Methodology
- UGVs are marked with ArUco markers, each with a unique ID. 
- UAV performs autonomous flight with downwards facing camera.
- Upon the detection of an ArUco marker, the pose of the UGV is estimated.
- A transform is published to this UGVs starting pose, and the map merging parameters are set.

### Obstacle Mapping Methodology
- UAV autonomously surveys the environment for obstacles with downwards-facing depth camera which produces a pointcloud.
- pointcloud is processed to isolate obstacle points.
- obstacle pointcloud is published to ROS server.
- UGVs add obstacle pointcloud to costmap for avoidance.

For detailed information about both of these methods, please read my [thesis](/Autonomous UAV-UGV Robot Collaboration for Exploration and Mapping of Unknown Environments.pdf)

## Pre-Requisites
- UAV (this project used a custom-built quadcopter running PX4 autopilot)
- Downwards-facing depth camera able to produce a pointcloud (Intel Realsense D435i with corresponding ROS package)
- UGV(s) (TUrtlebot3 Burgers were used)
- UGV 2D LiDAR
- [Aruco_detect library](https://github.com/BYUMarsRover/aruco_detect)
- [Open3D library](https://www.open3d.org/) and [open3d_conversions](https://github.com/marcoesposito1988/open3d_conversions) for ROS.


## Running this Package

Download this package and add it to your catkin_ws/src folder. Perform catkin build (or catkin make).

### Robot Localization
1. Roscore
2. Get UAV running with position estimate
3. Start UAV camera and ArUco detection topic
4. 
```
roslaunch mars-quadcopter turtle_tf_broadcaster.launch
```

Now when the UAV camera spots ArUco markers, it will create a pose estimate.

### Obstacle Mapping
1. Roscore
2. Get UAV running with position estimate
3. Publish UAV raw pointcloud topic
4. 
```
rosrun mars-quadcopter my_o3d_store_points
```

Now the isolated obstacles will be published to a topic called outlier_cloud

To get the UGVs to avoid the obstacles in this pointlcoud, their costmap parameter files must be updated. For the turtlebot this is under `turtlebot3_navigation/param/costmap_common_params.yaml`. Here, the [obstacles plugin](http://wiki.ros.org/costmap_2d/hydro/obstacles) should be used, and a new obstacle observation source should be added, using the outlier_cloud points. 

## What may need to be modified based on your setup
- ArUco marker ID numbers in [turtle_tf_broadcaster_1_avg.py](/src/robot localization/turtle_tf_broadcaster_1_avg.py) line 106.
- Depending on how many UGVs you are using. Versions of the above codes with the suffix _3 and for  3 UGVs rather than 2.
- Input pointcloud topic name in [my_o3d_store_points.py]('/src/obstacle mapping/my_o3d_store_points.py') in subscriber line 190.








