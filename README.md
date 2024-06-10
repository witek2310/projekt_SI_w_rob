# project AI in robotics

## steps to run environment:
### global packages to install:
- apt install ros-humble-gazebo-*
- sudo apt-get install ros-humble-turtlebot3*
### docker image:
https://hub.docker.com/repository/docker/jinlobana/humble_si/general  
to run docker, download an image from dockerhub and run ```start.sh``` script  
Make sure that Shared directory was the workspace directory.
### in workspace:
- colcon build (if needed)
- export TURTLEBOT3_MODEL=burger
- source /opt/ros/humble/setup.bash
- source install/setup.bash
- source /usr/share/gazebo/setup.bash
    - ros2 launch graph_cpp map_launch.py  
    - ros2 launch graph_cpp do_the_graphs.py
    - ros2 run turtlebot3_teleop teleop_keyboard

## project description

Project consist of two main ros2 nodes. First is turtle_location, which is extended external package. It was our first attempt to implement graph in python. 

### ```graph_location.cpp```

The GraphLocation node is designed to integrate data from various sensors (odometry, IMU, laser scan (which is used in AMCL)) to construct a graph-based representation of a robot's pose. This node utilizes the GTSAM (Georgia Tech Smoothing And Mapping) library for graph-based SLAM (Simultaneous Localization and Mapping) and sensor fusion.

#### Overview
The GraphLocation node subscribes to multiple topics to gather sensor data: 

1. /odom (nav_msgs/msg/Odometry): Odometry data providing the robot's pose and velocity.
2. /imu (sensor_msgs/msg/Imu): Inertial Measurement Unit data providing linear acceleration and angular velocity.
3. /laser (sensor_msgs/msg/LaserScan): Laser scan data, used for localization. (not used directly)
4. /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped): Pose estimated by the AMCL (Adaptive Monte Carlo Localization) node.  
The node publishes the fused pose with covariance information to a topic for visualisation in rviz2:
1. /graph_pose (geometry_msgs/msg/PoseWithCovarianceStamped): The estimated robot pose, including covariance, after sensor fusion using the graph-based SLAM approach.

#### Functionality
Initialization:  

Initializes subscriptions to the odometry, IMU, laser scan, and AMCL pose topics.
Sets up a publisher for the fused pose with covariance (/graph_pose).
Initializes the GTSAM graph, prior values, and noise models.
Sensor Callbacks:

Odometry Callback: Updates the current odometry pose and covariance.
IMU Callback: Integrates IMU measurements to update the preintegrated IMU state.
Laser Scan Callback: Stores the latest laser scan data.
AMCL Callback: Incorporates AMCL pose estimates into the graph, adds factors between consecutive poses, and optimizes the graph.
Graph Optimization:

Uses the Levenberg-Marquardt optimizer from GTSAM to optimize the pose graph.
Extracts the resulting pose and covariance and publishes them on the /graph_pose topic.
IMU Parameters
The imuParams function sets up the noise models for the IMU measurements, including:

Accelerometer noise
Gyroscope noise
Bias random walk noise for both accelerometer and gyroscope
Integration error covariance


## TODO:
- make robot visible in rviz
- add rviz to launch file 
