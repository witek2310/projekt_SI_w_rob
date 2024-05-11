# projekt SI w rob

# steps:
- apt install ros-humble-gazebo-*
- sudo apt-get install ros-humble-turtlebot3*
## in workspace:
- colcon build (if needed)
- export TURTLEBOT3_MODEL=burger
- source /opt/ros/humble/setup.bash
- source install/setup.bash
- gazebo maps/super.world
- ros2 run turtlebot3_teleop teleop_keyboard


# TODO:
- make a robot enviroment using GAZEBO
done
- make a robot itself
done
- make visible robot
- do the math :(
