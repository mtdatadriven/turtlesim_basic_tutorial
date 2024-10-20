#!/usr/bin/env python3  # This line indicates that the script should be run using the Python 3 interpreter.

import rclpy  # Import the ROS 2 client library for Python.
from rclpy.node import Node  # Import the Node class from the rclpy module to create a ROS node.
from turtlesim.msg import Pose, Color  # Import the Pose and Color message types from the turtlesim package.

class ReadTurtlesim(Node):  # Define a new class that inherits from Node to create a custom ROS node.
    def __init__(self):  # Initialize the class.
        super().__init__('Turtle1')  # Call the parent class constructor and set the node name to 'Turtle1'.
        
        # Subscribing to topics
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.callback_pose, 10)  # Subscribe to the /turtle1/pose topic to receive Pose messages and handle them with the callback_pose method.
        self.color_sub = self.create_subscription(Color, '/turtle1/color_sensor', self.callback_color, 10)  # Subscribe to the /turtle1/color_sensor topic to receive Color messages and handle them with the callback_color method.
        
        self.get_logger().info("Node started. Listening to /turtle1/pose and /turtle1/color_sensor topics...")  # Log a message indicating that the node has started and is listening to the specified topics.
    
    def callback_pose(self, data):  # Define a callback method to handle Pose messages.
        print("------------")  # Print a separator line for better readability.
        print(data)  # Print the received Pose data to the console.

    def callback_color(self, data):  # Define a callback method to handle Color messages.
        print("------------")  # Print a separator line for better readability.
        print(data)  # Print the received Color data to the console.

    def shutdown(self):  # Define a method to handle the shutdown of the node.
        self.get_logger().info("Stopping node...")  # Log a message indicating that the node is stopping.

def main(args=None):  # Define the main function.
    rclpy.init(args=args)  # Initialize the ROS 2 communication library.

    node = ReadTurtlesim()  # Create an instance of the ReadTurtlesim node.

    try:
        rclpy.spin(node)  # Keep the node running and processing callbacks until interrupted.
    except KeyboardInterrupt:  # Catch a keyboard interrupt (Ctrl+C).
        pass  # Do nothing on interrupt, allowing cleanup to proceed.
    finally:
        node.shutdown()  # Call the shutdown method to clean up.
        node.destroy_node()  # Destroy the node to free resources.
        rclpy.shutdown()  # Shutdown the ROS 2 library.

if __name__ == "__main__":  # Check if this script is being run directly.
    try:
        main()  # Call the main function.
    except Exception as e:  # Catch any exceptions that occur during execution.
        print(f"Error: {e}")  # Print the error message to the console.
        print("END of node")  # Indicate that the node has ended.
