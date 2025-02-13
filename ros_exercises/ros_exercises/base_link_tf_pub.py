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
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('base_link_tf_pub')

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1/20, self.timer_callback)


    def extract_tf(self, data):
        # Extract translation
        x = data.transform.translation.x
        y = data.transform.translation.y
        z = data.transform.translation.z

        # Extract quaternion
        qx = data.transform.rotation.x
        qy = data.transform.rotation.y
        qz = data.transform.rotation.z
        qw = data.transform.rotation.w

        rotation_matrix = tf_transformations.quaternion_matrix([qx, qy, qz, qw])[:3, :3]

        # Construct 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation_matrix  # Rotation
        T[:3, 3] = [x, y, z]  # Translation

        return T


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


    def timer_callback(self):
        left_to_odom = self.buffer.lookup_transform("odom", "left_cam", rclpy.time.Time())

        left_to_odom = self.extract_tf(left_to_odom)
        left_to_base = np.array([[1, 0, 0, 0], [0, 1, 0, 0.05], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
        base2_to_odom = self.create_tf("odom", "base_link_2", left_to_odom @ np.linalg.inv(left_to_base))

        self.broadcaster.sendTransform(base2_to_odom)
        self.get_logger().info(f"Broadcasted: {base2_to_odom.child_frame_id}")


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
