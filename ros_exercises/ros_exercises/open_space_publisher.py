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

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)

    def listener_callback(self, msg):
        distance = Float32()
        angle = Float32()

        ranges = msg.ranges
        index = max(range(len(ranges)), key = lambda i: ranges[i])

        distance.data = ranges[index]
        angle.data = index * msg.angle_increment

        self.distance_publisher.publish(distance)
        self.angle_publisher.publish(angle)
        self.get_logger().info(f"I heard and published: {distance.data}, {angle.data}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
