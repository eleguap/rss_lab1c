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

from custom_msgs.msg import OpenSpace
from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('open_space_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_topic', "open_space"),
                ('subscribe_topic', "fake_scan")
            ]
        )

        publish_topic, subscribe_topic = self.get_parameters(['publish_topic', 'subscribe_topic'])
        self.publish_topic, self.subscribe_topic = publish_topic.value, subscribe_topic.value

        self.subscription = self.create_subscription(
            LaserScan,
            self.subscribe_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(OpenSpace, self.publish_topic, 10)

    def listener_callback(self, msg):
        open_space = OpenSpace()

        ranges = msg.ranges
        index = max(range(len(ranges)), key = lambda i: ranges[i])

        open_space.angle = ranges[index]
        open_space.distance = index * msg.angle_increment

        self.publisher_.publish(open_space)
        self.get_logger().info(f"I heard and published: {open_space.angle}, {open_space.distance}")


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
