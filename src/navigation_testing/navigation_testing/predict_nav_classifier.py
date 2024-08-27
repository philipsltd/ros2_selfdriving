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
        self.model = load("src/mlmodels/xgboostClassifierModel.joblib")
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
        
        lidar_readings_selected = np.zeros((1, 6))

        left_readings = np.concatenate([lidar_readings[:20], lidar_readings[-20:]])

        min_distance = np.min(lidar_readings)
        max_distance = np.max(lidar_readings)
        average_distance = np.mean(lidar_readings)

        front_distance = np.min(lidar_readings[int(len(lidar_readings)*0.75) - 20:int(len(lidar_readings)*0.75 + 20)])
        left_distance = np.min(left_readings)
        right_distance = np.min(lidar_readings[int(len(lidar_readings)*0.5) - 20:int(len(lidar_readings)*0.5 + 20)])

        lidar_readings_selected[0, 0] = min_distance
        lidar_readings_selected[0, 1] = max_distance
        lidar_readings_selected[0, 2] = average_distance
        lidar_readings_selected[0, 3] = front_distance
        lidar_readings_selected[0, 4] = left_distance
        lidar_readings_selected[0, 5] = right_distance

        # Predict velocity using the model
        predicted_velocity = self.predict_velocity(lidar_readings_selected)

        # Publish the predicted velocity
        self.cmd_vel_publisher.publish(predicted_velocity)


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
