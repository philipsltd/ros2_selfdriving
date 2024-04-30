import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2  # Import PointCloud2 message

import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self, desired_topic, desired_bag_name):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=desired_bag_name,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name=desired_topic,
            type='sensor_msgs/msg/PointCloud2',  # Use PointCloud2 message type
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            PointCloud2,  # Use PointCloud2 message type for subscription
            desired_topic,
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        # Ensure message type matches (optional, can be removed)
        if isinstance(msg, PointCloud2):
            serialized_message = serialize_message(msg)
            self.writer.write(
                self.subscription.topic_name,  # Use subscription topic name
                serialized_message,
                self.get_clock().now().nanoseconds)
        else:
            print(f"Received unexpected message type: {type(msg)}")  # Optional: Log warning

def main(args=None):
    rclpy.init(args=args)
    print("Desired Topic:")
    desired_topic = input()
    print("Desired Topic: " + desired_topic + " selected.")
    print("Desired Bag Name:")
    desired_bag_name = input()
    sbr = SimpleBagRecorder(desired_topic, desired_bag_name)
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
