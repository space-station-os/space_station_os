#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3
from pymongo import MongoClient
import time

class MongoSubscriber(Node):
    def __init__(self):
        super().__init__('mongo_subscriber')

        self.client = MongoClient('mongodb://localhost:27017/')
        self.db = self.client['ssos_demo_db']

        self.imu_faulty_sub = self.create_subscription(
            Imu, '/ground_station/imu_faulty', self.imu_faulty_callback, 10)

        self.imu_cleaned_sub = self.create_subscription(
            Imu, '/ground_station/imu_cleaned', self.imu_cleaned_callback, 10)

        self.iss_pose_sub = self.create_subscription(
            PoseStamped, '/iss_position', self.iss_pose_callback, 10)

        self.torque_control_sub = self.create_subscription(
            Vector3, '/gnc/torque_control', self.torque_control_callback, 10)

        self.angvel_body_sub = self.create_subscription(
            Vector3, '/gnc/angvel_body', self.angvel_body_callback, 10)

    def imu_faulty_callback(self, msg):
        doc = {
            'timestamp': time.time(),
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'status': 'faulty'
        }
        self.db.imu.insert_one(doc)
        self.get_logger().info(f'[Mongo] Inserted Faulty IMU doc: {doc}')

    def imu_cleaned_callback(self, msg):
        doc = {
            'timestamp': time.time(),
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'status': 'cleaned'
        }
        self.db.imu.insert_one(doc)
        self.get_logger().info(f'[Mongo] Inserted Cleaned IMU doc: {doc}')

    def iss_pose_callback(self, msg):
        doc = {
            'timestamp': time.time(),
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.db.iss_pose.insert_one(doc)
        self.get_logger().info(f'[Mongo] Inserted ISS Pose doc: {doc}')

    def torque_control_callback(self, msg):
        doc = {
            'timestamp': time.time(),
            'torque_control': {
                'x': msg.x,
                'y': msg.y,
                'z': msg.z
            }
        }
        self.db.torque_control.insert_one(doc)
        self.get_logger().info(f'[Mongo] Inserted Torque Control doc: {doc}')

    def angvel_body_callback(self, msg):
        doc = {
            'timestamp': time.time(),
            'angular_velocity_body': {
                'x': msg.x,
                'y': msg.y,
                'z': msg.z
            }
        }
        self.db.angular_velocity_body.insert_one(doc)
        self.get_logger().info(f'[Mongo] Inserted Angular Velocity Body doc: {doc}')

def main(args=None):
    rclpy.init(args=args)
    node = MongoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
