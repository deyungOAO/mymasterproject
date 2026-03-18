import csv
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Initialize ROS 2 (required to deserialize messages properly)
rclpy.init()

# Bag file path
bag_path = '/home/ubuntu/AGV_ws/src/robot_description/description/diagrams/ros2_bags/joint_states'

# Set up reader with storage and converter options
reader = SequentialReader()
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage_options, converter_options)

# Get all topics and types
topics = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topics}

# Create CSV
with open('joint_states_positions.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (sec)', 'Position[2]', 'Position[3]'])

    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/joint_states':
            if count % 10 == 0:
                msg = deserialize_message(data, JointState)
                time_sec = t / 1e9

                if len(msg.position) > 3:
                    pos_2 = msg.position[2]
                    pos_3 = msg.position[3]
                    writer.writerow([time_sec, pos_2, pos_3])
            count += 1

