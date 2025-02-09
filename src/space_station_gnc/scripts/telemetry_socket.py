#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
from geometry_msgs.msg import Vector3, PoseStamped  # Replace with the appropriate message type

# Yamcs connection settings
YAMCS_IP = "127.0.0.1"
YAMCS_PORT = 10015

class Ros2YamcsBridge(Node):
    def __init__(self):
        super().__init__('ros2_to_yamcs_bridge')

        # angular velocity subscriber
        self.angvel_subscription = self.create_subscription(
            Vector3,                  
            '/angvel_body',       
            self.angvel_callback,
            10                                )
        # self.subscription  # prevent unused variable warning

        # pose subscriber
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/pose_all',
            self.pose_callback,
            10
        )

        # torque subscriber
        self.torque_subscription = self.create_subscription(
            Vector3,
            '/torque_all',
            self.torque_callback,
            10
        )
        # Create UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info("ROS 2 to Yamcs bridge initialized and running...")

    def angvel_callback(self, msg):
        self.angvel = msg

    def pose_callback(self, msg):
        # meta information
        self.timestamp = msg.header.stamp
        self.frame = msg.header.frame_id
        
        # pose information
        self.position = msg.pose.position
        self.orientation = msg.pose.quaternion

    def torque_callback(self, msg):
        self.torque = msg


    def ros_to_yamcs_callback(self):
        """
        Callback function to process data from the ROS 2 topic and send it to Yamcs.
        """
        try:
            # Get current timestamp in milliseconds
            timestamp = self.get_clock().now().nanoseconds // 1_000_000

            yamcs_data = struct.pack(">qfff", int(timestamp), self.angvel)  # Big-endian: int64 + float32

            # Send data to Yamcs via UDP
            self.udp_socket.sendto(yamcs_data, (YAMCS_IP, YAMCS_PORT))
            self.get_logger().info(f"Sent data to Yamcs: timestamp={timestamp}")

        except Exception as e:
            self.get_logger().error(f"Error sending data to Yamcs: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros2_to_yamcs_bridge = Ros2YamcsBridge()

    try:
        rclpy.spin(ros2_to_yamcs_bridge)
    except KeyboardInterrupt:
        ros2_to_yamcs_bridge.get_logger().info("Shutting down ROS 2 to Yamcs bridge...")

    # Destroy node explicitly (optional)
    ros2_to_yamcs_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
