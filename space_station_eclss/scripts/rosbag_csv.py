#!/usr/bin/env python3
import os
import rosbag2_py
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_rosbag(bag_path, topic_name):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    topic_type_dict = {topic.name: topic.type for topic in topics_and_types}

    if topic_name not in topic_type_dict:
        raise ValueError(f"Topic '{topic_name}' not found in the bag file.")

    message_type = get_message(topic_type_dict[topic_name])  # Get message type from topic name

    columns = ['timestamp', 'co2_mass', 'moisture_content', 'contaminants']
    data = []

    while reader.has_next():
        (topic, msg, timestamp) = reader.read_next()
        if topic == topic_name:
            deserialized_msg = deserialize_message(msg, message_type)
            try:
                data.append([
                    timestamp,
                    deserialized_msg.co2_mass,
                    deserialized_msg.moisture_content,
                    deserialized_msg.contaminants
                ])
            except AttributeError:
                print(f"Skipping a message due to incorrect data format at timestamp: {timestamp}")
                continue

    return pd.DataFrame(data, columns=columns)


if __name__ == "__main__":
    bag_path = '/home/siddarth/ros2ws/cycles7'  # Path to your bag file
    topic_name = '/adsorbent_air_quality'

    df = read_rosbag(bag_path, topic_name)
    output_csv = os.path.join(bag_path, 'adsorbent_air_quality.csv')
    df.to_csv(output_csv, index=False)
    print(f"Data saved to {output_csv}")
