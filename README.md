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

## TODO:
- make robot visible in rviz
- add rviz to launch file 
