// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GraphLocation : public rclcpp::Node
{
public:
  GraphLocation()
  : Node("graph_location")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&GraphLocation::odom_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&GraphLocation::imu_callback, this, std::placeholders::_1));
    laser_sub_= this->create_subscription<sensor_msgs::msg::LaserScan>("laser", 10, std::bind(&GraphLocation::laser_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "I heard: xd'");
    this->odom_msg = *msg;
    this->odom_recived = true;
  }
  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg){
    this->imu_msg = *msg;
    this->imu_recived = true;
  }
  void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg){
    this->laser_msg = *msg;
    this->laser_recived = true;
  }

  bool  odom_recived, imu_recived, laser_recived;
  sensor_msgs::msg::Imu imu_msg;
  sensor_msgs::msg::LaserScan laser_msg;
  nav_msgs::msg::Odometry odom_msg;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphLocation>());
  rclcpp::shutdown();
  return 0;
}


