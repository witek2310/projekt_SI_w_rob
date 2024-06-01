# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from pgmpy.models import MarkovModel
from pgmpy.factors.discrete import DiscreteFactor
from pgmpy.inference import Mplp


class GraphOptimazer(Node):

    def __init__(self):
        super().__init__('graph_optimisation')

        self.sub_imu = self.create_subscription(Imu,'imu',self.imu_callback,10); self.sub_imu  # prevent unused variable warning
        self.imu = Imu()

        self.sub_odom = self.create_subscription(Odometry,'odom',self.odom_callback,10); self.sub_odom # prevent unused variable warning
        self.odom = Odometry()

        self.sub_scan = self.create_subscription(LaserScan,'scan',self.scan_callback,10); self.sub_scan # prevent unused variable warning
        self.scan = LaserScan()

        self.graph = MarkovModel()
        self.graph
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def imu_callback(self, msg:Imu):
        self.imu = msg
        #self.get_logger().info(f'Imu: {self.imu.orientation}')
    def odom_callback(self, msg:Odometry):
        self.odom = msg
        #self.get_logger().info(f'Odom: {self.odom._pose}')
    def scan_callback(self, msg:LaserScan):
        self.scan = msg
        self.get_logger().info(f'Scan: {self.scan.ranges}')

    def timer_callback(self):
        pass


    


def main(args=None):
    rclpy.init(args=args)

    subscriber = GraphOptimazer()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
