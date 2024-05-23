import pandas as pd
from ros2bag import Reader

def bag_to_csv(bag_file, topic_name, output_file):
    """
    Converts ROS2 bag to pandas dataframe and saves as CSV.

    Args:
        bag_file (str): Path to the ROS2 bag file.
        topic_name (str): Name of the topic to extract data from.
        output_file (str): Path to save the CSV file.
    """
    # Initialize empty dataframe
    df = pd.DataFrame()
    
    # Create ROS2 bag reader
    reader = Reader(bag_file)

    # Iterate through messages in the bag
    for timestamp, topic, message in reader:
        # Check if message is from desired topic
        if topic == topic_name:
        # Extract data from message based on message type
            if topic == "/scan":  # sensor_msgs/LaserScan message
                ranges = message.ranges  # Extract range readings
                intensities = message.intensities  # Extract intensity readings (if available)
                # Create dictionary for scan data
                scan_data = {"timestamp": timestamp, "ranges": ranges, "intensities": intensities}
            elif topic == "/cmd_vel":  # geometry_msgs/Twist message
                linear_vel = message.linear.x  # Extract linear X velocity
                angular_vel = message.angular.z  # Extract angular Z velocity
                # Create dictionary for cmd_vel data
                cmd_vel_data = {"timestamp": timestamp, "linear_vel": linear_vel, "angular_vel": angular_vel}

    # Check if data is collected
    if not df.empty:
        # Save dataframe to CSV
        df.to_csv(output_file, index=False)
        print(f"Converted bag to CSV: {output_file}")

def main():
    # Prompt user for bag file name
    bag_file = input("Enter the path to your ROS2 bag file: ")

    # Create empty dataframes for each topic
    scan_df = pd.DataFrame()
    cmd_vel_df = pd.DataFrame()

    # Process scan data
    scan_output_file = "scan_data.csv"
    bag_to_csv(bag_file, "/scan", scan_output_file)

    # Process cmd_vel data
    cmd_vel_output_file = "cmd_vel_data.csv"
    bag_to_csv(bag_file, "/cmd_vel", cmd_vel_output_file)

if __name__ == "__main__":
    main()
