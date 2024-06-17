# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan  # Import PointCloud2 message\n",
from geometry_msgs.msg import Twist  # Import geometry_msgs.msg.Twist\n",
from sklearn.preprocessing import LabelEncoder

# Regular Imports

import numpy as np
import pandas as pd
from joblib import load
import json


class Nav_Prediction(Node):
    def __init__(self):
        super().__init__('nav_prediction')

        # Load the model
        self.model = load("src/mlmodels/randomForestModel.joblib")
        self.label_encoder = load("src/mlmodels/labelEncoder.joblib") # Load the label encoder to decode the predictions

        self.lidar_subscription = self.create_subscription(
            LaserScan,
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
        lidar_readings = list(msg.ranges)

        # Handle infinite values returned by the LiDAR sensor
        lidar_readings = [1e6 if x == float('inf') else x for x in lidar_readings]

        # Convert the readings to a string and remove brackets
        lidar_readings_str = str(lidar_readings).replace('[', '').replace(']', '')

        # Split the string into separate elements and convert them to floats
        lidar_readings_columns = [float(x) for x in lidar_readings_str.split(',')]

        # Convert to the format expected by the model (2D array)
        lidar_readings_columns = [lidar_readings_columns]

        # Predict velocity using the model
        predicted_velocity = self.predict_velocity(lidar_readings_columns)

        # Publish the predicted velocity
        self.cmd_vel_publisher.publish(predicted_velocity)  # Implement this if needed


    def predict_velocity(self, lidar_readings):
        # Process the LiDAR data (reshape if necessary)

        # Predict velocity using the model
        predicted_velocity_array = self.model.predict(lidar_readings)
        predicted_velocity = predicted_velocity_array[0]

        # Create a Twist message and set predicted values
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(predicted_velocity)

        print(twist_msg)

        return twist_msg


def main(args=None):
    rclpy.init(args=args)
    nav_prediction = Nav_Prediction()
    rclpy.spin(nav_prediction)
    nav_prediction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()