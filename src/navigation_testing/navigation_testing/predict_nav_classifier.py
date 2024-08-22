# ROS2 Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Import LaserScan message
from geometry_msgs.msg import Twist  # Import geometry_msgs.msg.Twist
from sklearn.preprocessing import LabelEncoder

# Regular Imports
import numpy as np
from joblib import load
import json

class Nav_Prediction(Node):
    def __init__(self):
        super().__init__('nav_prediction')

        # Load the model
        self.model = load("src/mlmodels/randomForestModel.joblib")
        self.label_encoder = load("src/mlmodels/labelEncoder.joblib")  # Load the label encoder to decode the predictions

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your actual LiDAR topic name
            self.lidar_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # cmd_vel topic name
            10
        )

    def lidar_callback(self, msg):
        # Process LiDAR data (extract ranges and potentially preprocess)
        lidar_readings = list(msg.ranges)

        # Handle infinite values returned by the LiDAR sensor
        lidar_readings = [1e6 if x == float('inf') else x for x in lidar_readings]

        # Down-sample the LiDAR readings from 1080 to 300 features
        downsampled_readings = self.downsample_lidar_readings(lidar_readings, 60)

        # Convert the readings to a 2D array as expected by the model
        lidar_readings_array = np.array(downsampled_readings).reshape(1, -1)

        # Predict velocity using the model
        predicted_velocity = self.predict_velocity(lidar_readings_array)

        # Publish the predicted velocity
        self.cmd_vel_publisher.publish(predicted_velocity)

    def downsample_lidar_readings(self, readings, target_length):
        factor = len(readings) / target_length
        downsampled = [readings[int(i * factor)] for i in range(target_length)]
        return downsampled

    def predict_velocity(self, lidar_readings):
        # Predict velocity using the model
        predicted_velocity = self.model.predict(lidar_readings)

        # Decode the predicted values
        predicted_velocity_decoded = self.label_encoder.inverse_transform(predicted_velocity)
        velocity_str = predicted_velocity_decoded[0]

        # Convert the string to a JSON dictionary
        velocity_dict = json.loads(velocity_str)

        # Create a Twist message and set predicted values
        twist_msg = Twist()
        twist_msg.linear.x = float(velocity_dict['linear']['x'])
        twist_msg.linear.y = float(velocity_dict['linear']['y'])
        twist_msg.linear.z = float(velocity_dict['linear']['z'])
        twist_msg.angular.x = float(velocity_dict['angular']['x'])
        twist_msg.angular.y = float(velocity_dict['angular']['y'])
        twist_msg.angular.z = float(velocity_dict['angular']['z'])

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
