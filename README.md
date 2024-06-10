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

Project consist of one main ros2 package. Inside are two launch files, which run two nodes. First node boot gazebo with custom maze map and turtlebot Burger. The second one subscribes to the topics from sensors in the first, makes a graph, optimises it and publish estimated pose with covariance. We use also rviz2 for visualisation and the last node is teleop, to control turtlebot in gazebo. Main program has been written in ```graph_location.cpp``` inside ```amcl_callback```.  

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

The graph looks like:

![graph](graph.svg)

Graph is updated every time the amcl collback is called. Why?\
Beacouse this is the slowest publishing topic. The information from the rest of topics are store in member variables and updated every time the topic related callback is called.\

#### Functionality
Initialization:  

Initializes subscriptions to the odometry, IMU, laser scan, and AMCL pose topics.
Sets up a publisher for the fused pose with covariance (/graph_pose).
Initializes the GTSAM graph, prior values, and noise models.  

Sensor Callbacks:  

- Odometry Callback: Updates the current odometry pose and covariance.
- IMU Callback: Integrates IMU measurements to update the preintegrated IMU state.
- Laser Scan Callback: Stores the latest laser scan data.
- AMCL Callback: Incorporates AMCL pose estimates into the graph, synchronise all measurements from sensors, adds factors for sensor in the right place to graph between consecutive poses, and optimizes the graph.  

Graph Optimization:  

## TODO IMAGE OF GRAPH

We use the Levenberg-Marquardt optimizer from GTSAM to optimize the pose graph.  

Extracts the resulting pose and covariance and publishes them on the /graph_pose topic. Covariance is calculated in the graph itself, we extract it and save to our PoseWithCovarianceStamped message. Few covariances (f.e. in z-z axis) are set to zero, because we have localisation in 2D not in 3D. 

```cpp
auto result = LevenbergMarquardtOptimizer(*this->graph_, this->initial_values_).optimize();
auto covariance = Marginals(*graph_, result);
auto one_cov = covariance.marginalCovariance(B(iteration_ + 1));
```



## TODO:
- add noise to /odom covariance to better imitate true measurements
- make robot visible in rviz
- add rviz to launch file 
