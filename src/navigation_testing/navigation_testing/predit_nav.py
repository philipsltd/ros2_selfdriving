# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2  # Import PointCloud2 message\n",
from geometry_msgs.msg import Twist  # Import geometry_msgs.msg.Twist\n",

# ML Imports

import numpy as np
import pandas as pd
from joblib import load


class Nav_Prediction(Node):
    def __init__(self):
        super().__init__('nav_prediction')

        # Load the model
        self.model = load("src/mlmodels/randomForestModel.joblib")

        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/scan',  # Replace with your actual LiDAR topic name
            self.lidar_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # cmd_vel topic name
            10)

    # TODO - Callback function for LiDAR data and execution of the prediction

    def lidar_callback(self, msg):
        # Process LiDAR data (extract X, Y, Z and potentially preprocess)
        lidar_data = msg.data # TODO - Your logic to extract and process relevant data from PointCloud2

        # Predict velocity using the model
        predicted_velocity = self.predict_velocity(lidar_data)

        # Publish the predicted velocity
        self.cmd_vel_publisher.publish(predicted_velocity)  # Implement this if needed


    def predict_velocity(self, lidar_data):
        # Process the LiDAR data (reshape if necessary)
        processed_data = lidar_data.data # TODO - Your logic to reshape data for model prediction

        # Predict velocity using the model
        predicted_velocity = self.model.predict(processed_data)

        # Create a Twist message and set predicted values
        twist_msg = Twist()
        twist_msg.linear.x = predicted_velocity[0]
        twist_msg.linear.y = predicted_velocity[1]
        twist_msg.linear.z = predicted_velocity[2]
        twist_msg.angular.x = predicted_velocity[3]
        twist_msg.angular.y = predicted_velocity[4]
        twist_msg.angular.z = predicted_velocity[5]

        return twist_msg


def main(args=None):
    rclpy.init(args=args)
    nav_prediction = Nav_Prediction()
    rclpy.spin(nav_prediction)
    nav_prediction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()