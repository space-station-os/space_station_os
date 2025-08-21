import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from importlib import import_module
import yaml
import os
from queue import Queue
import asyncio
import websockets
import json
import base64
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from spacepackets.ccsds import SpacePacketHeader, PacketType, SequenceFlags
qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE
        )
class TelemetryEncoderNode(Node):
    def __init__(self):
        super().__init__('telemetry_encoder')
        self.declare_parameter('apid_config', 'config/apid_mapping.yaml')
        self.declare_parameter('starlink_uri', 'ws://localhost:8080')  

        self.packet_queue = Queue(maxsize=1000)
        self.seq_count = 0

        config_path = self.get_parameter('apid_config').get_parameter_value().string_value
        self.starlink_uri = self.get_parameter('starlink_uri').get_parameter_value().string_value
        self.mapping = self.load_apid_mapping(config_path)

        self.subscribers = []
        self.uplink_pub = self.create_publisher(Bool, "/comms/uplink", qos)

        self.create_subscribers()

    def load_apid_mapping(self, path):
        with open(path, 'r') as file:
            raw = yaml.safe_load(file)
            return [entry for entry in raw if entry['communication_type'] == 'TM']

    def create_subscribers(self):
        for entry in self.mapping:
            topic = entry['ros_topic_name']
            type_str = entry['ros_type_name']
            apid = entry['packet_apid']
            msg_type = self.resolve_msg_type(type_str)

            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, apid=apid, topic=topic: self.encode_and_queue(msg, apid, topic),
                10
            )
            self.subscribers.append(sub)
            self.get_logger().info(f"Subscribed to {topic} with APID {apid}")

    def resolve_msg_type(self, type_str):
        pkg, msg_type = type_str.split('/msg/')
        return getattr(import_module(f"{pkg}.msg"), msg_type)

    def encode_and_queue(self, msg, apid, topic):
        serialized = serialize_message(msg)
        header = SpacePacketHeader(
            packet_type=PacketType.TM,
            sec_header_flag=False,
            apid=apid,
            seq_flags=SequenceFlags.UNSEGMENTED,
            seq_count=self.seq_count,
            data_len=len(serialized) - 1
        )
        self.seq_count = (self.seq_count + 1) % 16384

        packet_bytes = header.pack() + serialized

        encoded = base64.b64encode(packet_bytes).decode('utf-8')
        out_msg = {
            "topic": topic,
            "apid": apid,
            "timestamp": time.time(),
            "ccsds_encoded": encoded
        }

        if not self.packet_queue.full():
            self.packet_queue.put(json.dumps(out_msg))
        else:
            self.get_logger().warn("Packet queue full — dropping packet")

    def get_next_packet(self):
        if self.packet_queue.empty():
            return None
        return self.packet_queue.get()

    def publish_uplink_status(self, status: bool):
        msg = Bool()
        msg.data = status
        self.uplink_pub.publish(msg)
        self.get_logger().info(f"[Uplink Status] {'Connected' if status else 'Disconnected'}")

    async def ws_sender_loop(self):
        await asyncio.sleep(1.0)  
        self.get_logger().info(f"Starlink sender loop started. Target: {self.starlink_uri}")

        while rclpy.ok():
            try:
                self.get_logger().info(f"Trying to connect to Starlink WebSocket at {self.starlink_uri}")
                async with websockets.connect(self.starlink_uri) as ws:
                    self.get_logger().info(" Connected to Starlink WebSocket")
                    self.publish_uplink_status(True)

                    while rclpy.ok():
                        packet = self.get_next_packet()
                        if packet is None:
                            await asyncio.sleep(0.1)
                            continue

                        await ws.send(packet)
                        self.get_logger().info(f" Sent packet: {packet[:80]}...")
            except Exception as e:
                self.get_logger().warn(f" Starlink unreachable. Retrying in 3s... ({e})")
                self.publish_uplink_status(False)
                await asyncio.sleep(3.0)

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryEncoderNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(node.ws_sender_loop())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
