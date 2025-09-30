# ground_receiver_node.py

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from importlib import import_module
from spacepackets.ccsds import SpacePacketHeader
import yaml
import asyncio
import websockets
import json
import base64
import threading

class GroundReceiverNode(Node):
    def __init__(self):
        super().__init__('ground_receiver_node')
        self.declare_parameter('apid_config', 'config/apid_mapping.yaml')
        self.declare_parameter('forward_uri', 'ws://localhost:9090')
        self.declare_parameter('receive_uri', 'ws://localhost:8081')

        config_path = self.get_parameter('apid_config').get_parameter_value().string_value
        self.forward_uri = self.get_parameter('forward_uri').get_parameter_value().string_value
        self.receive_uri = self.get_parameter('receive_uri').get_parameter_value().string_value

        self.apid_map = self.load_apid_mapping(config_path)

    def load_apid_mapping(self, path):
        with open(path, 'r') as f:
            raw = yaml.safe_load(f)
            return {
                entry['packet_apid']: {
                    'ros_type': entry['ros_type_name'],
                    'topic': entry['ros_topic_name']
                } for entry in raw if entry['communication_type'] == 'TM'
            }

    def resolve_msg_type(self, type_str):
        pkg, msg_type = type_str.split('/msg/')
        return getattr(import_module(f"{pkg}.msg"), msg_type)

    def decode_packet(self, encoded_msg):
        parsed = json.loads(encoded_msg)
        apid = parsed['apid']
        raw = base64.b64decode(parsed['ccsds_encoded'])
        timestamp = parsed['timestamp']
        topic = parsed['topic']

        header = SpacePacketHeader.unpack(raw[:6])
        payload = raw[6:]

        info = self.apid_map.get(apid)
        if not info:
            self.get_logger().warn(f"Unknown APID: {apid}")
            return None, None, None

        ros_type_str = info['ros_type']
        msg_type = self.resolve_msg_type(ros_type_str)
        ros_msg = deserialize_message(payload, msg_type)
        return topic, ros_msg, timestamp

    async def forward_to_openmct(self, topic, msg, timestamp):
        try:
            async with websockets.connect(self.forward_uri) as ws:
                await ws.send(json.dumps({
                    "topic": topic,
                    "value": getattr(msg, 'data', str(msg)),
                    "timestamp": timestamp
                }))
                self.get_logger().info(f"[FORWARD] Sent to OpenMCT: {topic}")
        except Exception as e:
            self.get_logger().warn(f"[FORWARD] Failed to send: {e}")

    async def websocket_listener(self):
        self.get_logger().info(f"[GROUND] Listening to Starlink on {self.receive_uri}")
        while rclpy.ok():
            try:
                async with websockets.connect(self.receive_uri) as ws:
                    self.get_logger().info("[GROUND] Connected to Starlink WebSocket")
                    async for msg in ws:
                        try:
                            topic, decoded_msg, ts = self.decode_packet(msg)
                            if topic is not None:
                                self.get_logger().info(
                                    f"[GROUND] Received ➜ Topic: {topic} ➜ Msg: {decoded_msg}"
                                )
                                await self.forward_to_openmct(topic, decoded_msg, ts)
                        except Exception as e:
                            self.get_logger().error(f"[GROUND] Failed to decode: {e}")
            except Exception as e:
                self.get_logger().warn(f"[GROUND] Starlink connection failed: {e}")
                await asyncio.sleep(3.0)

def main(args=None):
    rclpy.init(args=args)
    node = GroundReceiverNode()

    # ROS in background
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Async loop in main thread
    try:
        asyncio.run(node.websocket_listener())
    except KeyboardInterrupt:
        node.get_logger().info("GroundReceiverNode interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
