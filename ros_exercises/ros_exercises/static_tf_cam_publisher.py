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
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_cam_publisher')

        self.static_br = StaticTransformBroadcaster(self)

        left_T = np.array([[1, 0, 0, 0], [0, 1, 0, 0.05], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
        left = self.create_tf("base_link", "left_cam", left_T)
        right_T = np.array([[1, 0, 0, 0], [0, 1, 0, -0.10], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
        right = self.create_tf("left_cam", "right_cam", right_T)

        self.static_br.sendTransform(left)
        self.static_br.sendTransform(right)
        self.get_logger().info(f"Broadcasted: {left.child_frame_id}, {right.child_frame_id}")

    def create_tf(self, parent, child, T):
        tf = TransformStamped()

        # Convert rotation matrix to quaternion
        translation = T[:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(T)

        # Set the header
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child

        # Set the translation and quaternion
        tf.transform.translation.x = translation[0]
        tf.transform.translation.y = translation[1]
        tf.transform.translation.z = translation[2]

        tf.transform.rotation.x = quaternion[0]
        tf.transform.rotation.y = quaternion[1]
        tf.transform.rotation.z = quaternion[2]
        tf.transform.rotation.w = quaternion[3]

        return tf


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
