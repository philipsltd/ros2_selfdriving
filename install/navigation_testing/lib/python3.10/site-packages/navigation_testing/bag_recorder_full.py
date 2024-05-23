import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Twist  # Import geometry_msgs.msg.Twist

import rosbag2_py


class SimpleBagRecorder(Node):
    def __init__(self, desired_bag_name):
        super().__init__('bag_recorder_full')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=desired_bag_name,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Topic metadata for LiDAR data (assuming PointCloud2)
        lidar_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/scan',  # Replace with your actual LiDAR topic name
            type='sensor_msgs/msg/LaserScan',
            serialization_format='cdr')

        # Topic metadata for cmd_vel
        cmd_vel_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/cmd_vel',  # Replace with the actual cmd_vel topic name
            type='geometry_msgs/msg/Twist',
            serialization_format='cdr')

        self.writer.create_topic(lidar_topic_info)
        self.writer.create_topic(cmd_vel_topic_info)

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your actual LiDAR topic name
            self.lidar_callback,
            10)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with the actual cmd_vel topic name
            self.cmd_vel_callback,
            10)

        self.last_record_time = self.get_clock().now() # Initial timestamp

    def lidar_callback(self, msg):
        # Check if 200ms has passed since the last recording
        current_time = self.get_clock().now()
        if (current_time - self.last_record_time).nanoseconds / 1e9 >= 0.2:  # Check for 200ms interval
            # Process and write LiDAR data (PointCloud2 message)
            serialized_message = serialize_message(msg)
            self.writer.write('/scan',  # Replace with your actual LiDAR topic name
                            serialized_message,
                            self.get_clock().now().nanoseconds)  # Use node clock
            self.last_record_time = current_time

    def cmd_vel_callback(self, msg):
        # Similar logic for cmd_vel callback
        current_time = self.get_clock().now()
        if (current_time - self.last_record_time).nanoseconds / 1e9 >= 0.2:
            serialized_message = serialize_message(msg)
            self.writer.write('/cmd_vel',  # Replace with actual topic name if different
                            serialized_message,
                            current_time.nanoseconds)  # Use node clock
            self.last_record_time = current_time



def main(args=None):
    rclpy.init(args=args)
    print("Desired Bag Name:")
    desired_bag_name = input()
    sbr = SimpleBagRecorder(desired_bag_name)
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
