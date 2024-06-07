# projekt SI w rob

# steps:
- apt install ros-humble-gazebo-*
- sudo apt-get install ros-humble-turtlebot3*
## in workspace:
- colcon build (if needed)
- export TURTLEBOT3_MODEL=burger
- source /opt/ros/humble/setup.bash
- source install/setup.bash
- source /usr/share/gazebo/setup.bash
- gazebo maps/super.world  

    - ros2 launch graph_cpp map_launch.py  
    - ros2 launch graph_cpp do_the_graphs.py

- ros2 run turtlebot3_teleop teleop_keyboard


# TODO:
- covariance matrix
- path of robot 
- make robot to move in rviz
- make code beautiful and yummy
