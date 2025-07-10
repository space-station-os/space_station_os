#!/usr/bin/env python3

import binascii
import io
import socket
import sys
import argparse

from struct import unpack_from
from threading import Thread
from time import sleep

# ---------- ROS imports ----------
import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

parser = argparse.ArgumentParser(description='Yamcs Simulator + ROS CCSDS Bridge')
parser.add_argument('--testdata', type=str, default='testdata.ccsds', help='CCSDS test data file')
parser.add_argument('--tm_host', type=str, default='127.0.0.1', help='TM host')
parser.add_argument('--tm_port', type=int, default=10015, help='TM port')
parser.add_argument('-r', '--rate', type=int, default=1, help='TM playback rate (Hz)')
parser.add_argument('--tc_host', type=str, default='127.0.0.1', help='TC host')
parser.add_argument('--tc_port', type=int, default=10025, help='TC port')

# ------------------------------------------------------------------------
#                             SIMULATOR LOGIC
# ------------------------------------------------------------------------

def send_tm(simulator):
    tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with io.open(simulator.testdata, 'rb') as f:
        simulator.tm_counter = 1
        header = bytearray(6)
        while f.readinto(header) == 6:
            (length,) = unpack_from('>H', header, 4)
            packet = bytearray(length + 7)
            f.seek(-6, io.SEEK_CUR)
            f.readinto(packet)

            tm_socket.sendto(packet, (simulator.tm_host, simulator.tm_port))
            simulator.tm_counter += 1
            sleep(1 / simulator.rate)

def receive_tc(simulator):
    tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tc_socket.bind((simulator.tc_host, simulator.tc_port))
    while True:
        data, _ = tc_socket.recvfrom(4096)
        simulator.last_tc = data
        simulator.tc_counter += 1

class Simulator:
    def __init__(self, testdata, tm_host, tm_port, tc_host, tc_port, rate):
        self.testdata  = testdata
        self.tm_host   = tm_host
        self.tm_port   = tm_port
        self.tc_host   = tc_host
        self.tc_port   = tc_port
        self.rate      = rate

        self.tm_counter = 0
        self.tc_counter = 0
        self.last_tc    = None

        self.tm_thread  = None
        self.tc_thread  = None

    def start(self):
        self.tm_thread = Thread(target=send_tm, args=(self,))
        self.tm_thread.daemon = True
        self.tm_thread.start()

        self.tc_thread = Thread(target=receive_tc, args=(self,))
        self.tc_thread.daemon = True
        self.tc_thread.start()

    def print_status(self):
        cmdhex = None
        if self.last_tc:
            cmdhex = binascii.hexlify(self.last_tc).decode('ascii')
        return f'Sent: {self.tm_counter} packets. Received: {self.tc_counter} commands. Last command: {cmdhex}'

# ------------------------------------------------------------------------
#                           ROS BRIDGE LOGIC
# ------------------------------------------------------------------------
class ROSCcsdsBridgeNode(Node):
    def __init__(self, tm_host, tm_port):
        super().__init__('ros_ccsds_bridge')

        self.tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tm_host = tm_host
        self.tm_port = tm_port

        self.seq_count_imu   = 0
        self.seq_count_pose  = 0
        self.seq_count_clean = 0

        self.imu_sub = self.create_subscription(
            Imu,
            '/ground_station/imu_faulty',
            self.imu_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/iss_position',
            self.pose_callback,
            10
        )
        self.clean_sub = self.create_subscription(
            Float32,
            '/ground_station/imu_cleaned',
            self.clean_data_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ox, oy, oz, ow = (msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)

        payload = struct.pack('>7f', px, py, pz, ox, oy, oz, ow)
        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x124
        seq_flags_count = (3 << 14) | (self.seq_count_pose & 0x3FFF)
        self.seq_count_pose += 1
        pkt_length = len(payload) + 6 - 1

        header = struct.pack('>HHH', version_type_sec_apid, seq_flags_count, pkt_length)
        packet = header + payload
        self.tm_socket.sendto(packet, (self.tm_host, self.tm_port))
        self.get_logger().info(f"Sent Pose seq={self.seq_count_pose - 1}, pos=({px:.2f},{py:.2f},{pz:.2f})")

    def clean_data_callback(self, msg: Float32):
        val = msg.data
        payload = struct.pack('>f', val)
        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x125
        seq_flags_count = (3 << 14) | (self.seq_count_clean & 0x3FFF)
        self.seq_count_clean += 1
        pkt_length = len(payload) + 6 - 1

        header = struct.pack('>HHH', version_type_sec_apid, seq_flags_count, pkt_length)
        packet = header + payload
        self.tm_socket.sendto(packet, (self.tm_host, self.tm_port))
        self.get_logger().info(f"Sent Cleaned seq={self.seq_count_clean - 1}, val={val:.4f}")


    def imu_callback(self, msg: Imu):
        avx, avy, avz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        payload = struct.pack('>3f', avx, avy, avz)
        version_type_sec_apid = (0 << 13) | (0 << 12) | (0 << 11) | 0x123
        seq_flags_count = (3 << 14) | (self.seq_count_imu & 0x3FFF)
        self.seq_count_imu += 1
        pkt_length = len(payload) + 6 - 1

        header = struct.pack('>HHH', version_type_sec_apid, seq_flags_count, pkt_length)
        packet = header + payload
        self.tm_socket.sendto(packet, (self.tm_host, self.tm_port))
        self.get_logger().info(f"Sent IMU seq={self.seq_count_imu - 1}, av=({avx:.4f},{avy:.4f},{avz:.4f})")

    # (Pose and clean_data_callback remain unchanged as above)

# ------------------------------------------------------------------------
#                                MAIN
# ------------------------------------------------------------------------
def main():
    args = parser.parse_args()

    simulator = Simulator(args.testdata, args.tm_host, args.tm_port, args.tc_host, args.tc_port, args.rate)
    simulator.start()
    sys.stdout.write(f"Simulator running: TM host={args.tm_host}, port={args.tm_port}, TC host={args.tc_host}, port={args.tc_port}, rate={args.rate}Hz\n")

    rclpy.init()
    node = ROSCcsdsBridgeNode(args.tm_host, args.tm_port)

    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        prev_status = None
        while True:
            status = simulator.print_status()
            if status != prev_status:
                sys.stdout.write('\r' + status)
                sys.stdout.flush()
                prev_status = status
            sleep(0.5)
    except KeyboardInterrupt:
        sys.stdout.write('\nShutting down...\n')
        sys.stdout.flush()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()