#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

class SpaceStationPathNode(Node):
    def __init__(self):
        super().__init__('space_station_path_node')
        self.publisher = self.create_publisher(PoseStamped, 'iss_position', 10)
        self.subscription = self.create_subscription(
            Imu, 'synthetic_imu_data', self.imu_callback, 10
        )

        self.timer = self.create_timer(1.0, self.update_and_publish)

        # Start at 6778 (km), ~400 km above Earth radius 6378
        self.position = np.array([6778.0, 0.0, 0.0])
        # Identity quaternion
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])

        self.orbit_speed = 7.66  # km/s
        self.get_logger().info("SpaceStationPathNode started (Stable Orbit + IMU orientation).")

    def imu_callback(self, msg: Imu):
        # Keep orientation updated from IMU's angular velocity
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        dt = 0.1  # if your IMU is ~10 Hz
        delta_rot = R.from_rotvec([wx * dt, wy * dt, wz * dt])
        current_rot = R.from_quat(self.orientation)
        new_rot = delta_rot * current_rot
        self.orientation = new_rot.as_quat()

        self.get_logger().info(f"Updated orientation: {self.orientation}")

    def update_and_publish(self):
        # Rotate around Z-axis each second => stable circular orbit
        speed = self.orbit_speed
        r = np.linalg.norm(self.position)  # should remain ~6778
        dtheta = speed / r  # radians per second

        cosd = np.cos(dtheta)
        sind = np.sin(dtheta)
        rot_z = np.array([
            [cosd, -sind, 0],
            [sind,  cosd, 0],
            [0,     0,     1]
        ])
        self.position = rot_z @ self.position

        # Publish updated pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'earth_orbit'

        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])

        pose_msg.pose.orientation.x = float(self.orientation[0])
        pose_msg.pose.orientation.y = float(self.orientation[1])
        pose_msg.pose.orientation.z = float(self.orientation[2])
        pose_msg.pose.orientation.w = float(self.orientation[3])

        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Publishing station position: {self.position}")

def main(args=None):
    rclpy.init(args=args)
    node = SpaceStationPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
