#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# TM (telemetry) config
TM_HOST = '127.0.0.1'   # Yamcs listening address for telemetry
TM_PORT = 10015         # Yamcs listening port for TM

# TC (telecommand) config
TC_HOST = '127.0.0.1'   # Local IP to bind for receiving commands
TC_PORT = 10025         # Port to receive commands from Yamcs

class ROSCcsdsBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_ccsds_bridge')

        # Socket for sending telemetry
        self.tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Socket for receiving telecommands
        self.tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_socket.bind((TC_HOST, TC_PORT))
        self.tc_socket.setblocking(False)  # We'll poll in a timer callback

        # Track separate sequence counts for each APID
        self.seq_count_imu   = 0
        self.seq_count_pose  = 0
        self.seq_count_clean = 0

        # Subscribe to IMU faulty data
        self.imu_sub = self.create_subscription(
            Imu,
            '/ground_station/imu_faulty',
            self.imu_callback,
            10
        )

        # Subscribe to ISS position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/iss_position',
            self.pose_callback,
            10
        )

        # Subscribe to cleaned IMU data
        self.clean_sub = self.create_subscription(
            Float32,
            '/ground_station/imu_cleaned',
            self.clean_data_callback,
            10
        )

        # Create a timer to regularly check for telecommands (TC)
        self.tc_timer = self.create_timer(0.1, self.check_for_telecommands)

    def imu_callback(self, msg: Imu):
        """Build and send a CCSDS packet with IMU angular velocity (APID 0x123)."""
        avx = msg.angular_velocity.x
        avy = msg.angular_velocity.y
        avz = msg.angular_velocity.z

        payload = struct.pack('>3f', avx, avy, avz)

        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x123
        seq_flags_count = (3 << 14) | (self.seq_count_imu & 0x3FFF)
        self.seq_count_imu += 1

        pkt_length = len(payload) + 6 - 1
        header = struct.pack('>HHH',
                             version_type_sec_apid,
                             seq_flags_count,
                             pkt_length)

        ccsds_packet = header + payload

        self.tm_socket.sendto(ccsds_packet, (TM_HOST, TM_PORT))
        self.get_logger().info(f"Sent Faulty IMU TM Packet seq={self.seq_count_imu - 1}, av=({avx:.4f},{avy:.4f},{avz:.4f})")

    def pose_callback(self, msg: PoseStamped):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        payload = struct.pack('>7f', px, py, pz, ox, oy, oz, ow)

        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x124
        seq_flags_count = (3 << 14) | (self.seq_count_pose & 0x3FFF)
        self.seq_count_pose += 1

        pkt_length = len(payload) + 6 - 1
        header = struct.pack('>HHH', version_type_sec_apid, seq_flags_count, pkt_length)

        ccsds_packet = header + payload
        self.tm_socket.sendto(ccsds_packet, (TM_HOST, TM_PORT))
        self.get_logger().info(
            f"Sent Pose TM Packet seq={self.seq_count_pose - 1}, pos=({px:.2f},{py:.2f},{pz:.2f})"
        )

    def clean_data_callback(self, msg: Float32):
        val = msg.data

        payload = struct.pack('>f', val)

        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x125
        seq_flags_count = (3 << 14) | (self.seq_count_clean & 0x3FFF)
        self.seq_count_clean += 1

        pkt_length = len(payload) + 6 - 1
        header = struct.pack('>HHH', version_type_sec_apid, seq_flags_count, pkt_length)

        ccsds_packet = header + payload
        self.tm_socket.sendto(ccsds_packet, (TM_HOST, TM_PORT))
        self.get_logger().info(
            f"Sent Cleaned IMU Data Packet seq={self.seq_count_clean - 1}, val={val:.4f}"
        )

    def check_for_telecommands(self):
        try:
            data, addr = self.tc_socket.recvfrom(1024)
        except BlockingIOError:
            return

        self.get_logger().info(f"Received TC from {addr}, hex={data.hex()}")


def main(args=None):
    rclpy.init(args=args)
    node = ROSCcsdsBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
