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

import rclpy, math, random
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.scan_publisher = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.length_publisher = self.create_publisher(Float32, 'range_test', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = (-2/3) * math.pi
        scan.angle_max = (2/3) * math.pi
        scan.angle_increment = (1/300) * math.pi
        scan.scan_time = 1.0
        scan.range_min = 1.0
        scan.range_max = 10.0
        num_ranges = int((scan.angle_max - scan.angle_min)/scan.angle_increment) + 1
        scan.ranges = [random.uniform(scan.range_min, scan.range_max) for _ in range(num_ranges)]

        length = Float32()
        length.data = float(num_ranges)

        self.scan_publisher.publish(scan)
        self.length_publisher.publish(length)
        self.get_logger().info(f"Publishing: [{scan.ranges[0]} ... {scan.ranges[-1]}] and {length.data}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
