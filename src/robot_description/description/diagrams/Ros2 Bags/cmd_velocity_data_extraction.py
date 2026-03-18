import rclpy
import rosbag2_py
import csv
import os
from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message

# === CONFIGURE PATH TO YOUR BAG FILE ===
bag_path = '/home/ubuntu/AGV_ws/src/robot_description/description/diagrams/ros2_bags/cmd_velocity'  # change to your bag path

# Set up bag reader
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage_options, converter_options)

# Create CSV
output_csv = 'cmd_vel_data.csv'
with open(output_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (s)', 'Linear X', 'Angular Z'])

    # Load topic types
    type_map = {topic_info.name: topic_info.type for topic_info in reader.get_all_topics_and_types()}

    # Process messages
    count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/cmd_vel':
            msg_type = type_map[topic]
            msg = deserialize_message(data, Twist)
            time_sec = timestamp / 1e9  # convert from nanoseconds
            writer.writerow([time_sec, msg.linear.x, msg.angular.z])
            count += 1

