import rosbag2_py
import pickle
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/home/carlos/reach_bravo_7_ws/rosbag/rosbag2_2025_08_22-19_51_15"


pickle_path = "ros2_bag_22_Aug_1.pkl"

# Open bag
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)
reader.open(storage_options, converter_options)

# Map topics to message types
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

bag_data = []

while reader.has_next():
    topic, data, t = reader.read_next()
    msg_type_str = topic_types[topic]  # e.g. "sensor_msgs/msg/JointState"
    msg_type = get_message(msg_type_str)
    msg = deserialize_message(data, msg_type)
    bag_data.append({
        "topic": topic,
        "time": t,
        "msg": msg
    })

# Save to pickle
with open(pickle_path, "wb") as f:
    pickle.dump(bag_data, f)

print(f"Saved {len(bag_data)} messages to {pickle_path}")