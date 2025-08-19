import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import csv
import os
from datetime import datetime
import glob
from typing import List, Union, Dict
import sys


def read_rosbag_to_one_csv(bag_path: str, topic_names: List[str], out_dirpath: str) -> str:
    # Search for the database file (.db3)
    yaml_fpath = os.path.join(bag_path, 'metadata.yaml')
    parent_db_dname = os.path.basename(os.path.dirname(yaml_fpath))
    db_path = os.path.join(bag_path, parent_db_dname + '_0.db3')
    if not os.path.exists(db_path):
        print(f"rosbag2 database (.db3) not found: {db_path}")
        return

    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Retrieve and display the list of topics
    cursor.execute("SELECT id, name, type FROM topics")
    all_topics = cursor.fetchall()
    print("=== List of topics in rosbag ===")
    if len(all_topics) == 0:
        print('Empty rosbag file.')
        return
    
    for id_, name, type_ in all_topics:
        print(f"  ID: {id_}, Name: {name}, Type: {type_}")
    print("================================")
    print()

    # Create a dictionary: topic name → (id, type)
    topic_map = {name: (id_, type_) for id_, name, type_ in all_topics}

    # If the topic list is empty, use all topics
    if not topic_names:
        topic_names = list(topic_map.keys())
        print("Topic list is empty. All topics will be included.")

    column_value_list_dict: Dict[str, List[Union[str, int, float]]] = dict()

    for topic_name in topic_names:
        if topic_name not in topic_map:
            print(f"Topic {topic_name} does not exist in rosbag. Skipping.")
            continue

        value_list = []

        topic_id, msg_type = topic_map[topic_name]
        msg_module = get_message(msg_type)

        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
        rows = cursor.fetchall()

        if not rows:
            print(f"No data for topic {topic_name}.")
            continue

        # Remove the first "/"
        topic_name_base = topic_name[1:]

        # Set header based on message type
        header_list = []
        if msg_type == 'std_msgs/msg/Float64':
            header_list.append(topic_name_base)
        elif msg_type == 'std_msgs/msg/Int32':
            header_list.append(topic_name_base)
        elif msg_type == 'geometry_msgs/msg/Vector3':
            for s in ('x', 'y', 'z'):
                header_list.append(topic_name_base + '_' + s)
        elif msg_type == 'geometry_msgs/msg/Quaternion':
            for s in ('x', 'y', 'z', 'w'):
                header_list.append(topic_name_base + '_' + s)
        else:
            print(f"Unsupported type: {msg_type} → outputting as str(msg)")
            header_list.append(topic_name_base + '_raw')

        for header in header_list:
            column_value_list_dict[header] = []

        for _, data in rows:
            msg = deserialize_message(data, msg_module)

            if msg_type in ('std_msgs/msg/Float64', 'std_msgs/msg/Int32'):
                column_value_list_dict[header_list[0]].append(msg.data)
            elif msg_type == 'geometry_msgs/msg/Vector3':
                column_value_list_dict[header_list[0]].append(msg.x)
                column_value_list_dict[header_list[1]].append(msg.y)
                column_value_list_dict[header_list[2]].append(msg.z)
            elif msg_type == 'geometry_msgs/msg/Quaternion':
                column_value_list_dict[header_list[0]].append(msg.x)
                column_value_list_dict[header_list[1]].append(msg.y)
                column_value_list_dict[header_list[2]].append(msg.z)
                column_value_list_dict[header_list[3]].append(msg.w)
    print()

    # -------- Check the number of lines --------
    n_lines = None
    print('-------- The number of published values --------')
    for column, value_list in column_value_list_dict.items():
        next_n_lines = len(value_list)
        print('    {}: {}'.format(column, next_n_lines))
        if n_lines is None:
            n_lines = next_n_lines
        else:
            next_n_lines = len(value_list)
            if n_lines != next_n_lines:
                print('Not match!')

    csv_filename = parent_db_dname + '.csv'
    csv_filepath = os.path.join(out_dirpath, csv_filename)
    with open(csv_filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        column_list = [column for column in column_value_list_dict.keys()]
        writer.writerow(column_list)

        for i in range(n_lines):
            row_list = []
            for column, value_list in column_value_list_dict.items():
                row_list.append(value_list[i])
            writer.writerow(row_list)

    conn.close()

    return csv_filepath


def main():
    
    if len(sys.argv) == 1:
        # Directory path of rosbag2
        bag_fpath_list = glob.glob(r'./space_station_gnc/tests/orbit_dynamics/rosbag2_out/*')
        bag_fpath_list.sort()
        # The newest directory
        bag_path = bag_fpath_list[-1]
    elif len(sys.argv) == 2:
        bag_path = sys.argv[1]
    else:
        raise ValueError('Invlaid arguments.')

    print('Input rosbag: {}'.format(bag_path))

    out_dirpath = './space_station_gnc/tests/orbit_dynamics/result_csv'
    if not os.path.exists(out_dirpath):
        os.makedirs(out_dirpath, exist_ok=True)
    
    topic_list = []
    out_fpath = read_rosbag_to_one_csv(bag_path, topic_list, out_dirpath)
    print('Output CSV: {}'.format(out_fpath))


if __name__ == '__main__':
    main()
