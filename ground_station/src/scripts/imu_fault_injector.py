#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
import random

class IMUFaultInjector(Node):
    def __init__(self):
        super().__init__('imu_fault_injector')

        self.subscription = self.create_subscription(
            Vector3,
            '/gnc/angvel_body',  # active topic
            self.imu_callback,
            10)

        self.publisher = self.create_publisher(Imu, '/ground_station/imu_faulty', 10)

        self.missing_data_probability = 0.02
        self.outlier_probability = 0.05
        self.noise_stddev_angvel = 0.00001
        self.noise_stddev_linacc = 0.01

        self.get_logger().info(
            f"IMU Fault Injector Initialized: missing_data_probability={self.missing_data_probability}, "
            f"outlier_probability={self.outlier_probability}, noise_stddev_angvel={self.noise_stddev_angvel}, "
            f"noise_stddev_linacc={self.noise_stddev_linacc}")

    def imu_callback(self, msg: Vector3):
        if random.random() < self.missing_data_probability:
            self.get_logger().warn("Simulated missing IMU data.")
            return

        faulty_msg = Imu()
        faulty_msg.header.stamp = self.get_clock().now().to_msg()
        faulty_msg.header.frame_id = 'space_station'

        faulty_msg.angular_velocity.x = msg.x + np.random.normal(0, self.noise_stddev_angvel)
        faulty_msg.angular_velocity.y = msg.y + np.random.normal(0, self.noise_stddev_angvel)
        faulty_msg.angular_velocity.z = msg.z + np.random.normal(0, self.noise_stddev_angvel)

        faulty_msg.linear_acceleration.x = np.random.normal(0, self.noise_stddev_linacc)
        faulty_msg.linear_acceleration.y = np.random.normal(0, self.noise_stddev_linacc)
        faulty_msg.linear_acceleration.z = np.random.normal(0, self.noise_stddev_linacc)

        if random.random() < self.outlier_probability:
            anomaly_axis = random.choice(['x', 'y', 'z'])
            setattr(faulty_msg.angular_velocity, anomaly_axis, random.uniform(-1, 1))
            self.get_logger().warn(f"Injected anomaly in angular velocity axis: {anomaly_axis}")

        self.publisher.publish(faulty_msg)

        self.get_logger().info(
            f"Published faulty IMU data: AngVel(x={faulty_msg.angular_velocity.x:.5f}, "
            f"y={faulty_msg.angular_velocity.y:.5f}, "
            f"z={faulty_msg.angular_velocity.z:.5f}), "
            f"LinAcc(x={faulty_msg.linear_acceleration.x:.5f}, "
            f"y={faulty_msg.linear_acceleration.y:.5f}, "
            f"z={faulty_msg.linear_acceleration.z:.5f})")

def main(args=None):
    rclpy.init(args=args)
    node = IMUFaultInjector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
