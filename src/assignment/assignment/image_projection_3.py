import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
# Define the PoseSubscriber class, inheriting from the Node class
class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
                # Create a subscriber listening to PoseStamped messages on the '/limo/object_location' topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/limo/object_location',
            self.posestamped_callback,
            10
        )
        self.coordinates = []
            # Callback function called when a message is received
    def posestamped_callback(self, msg):
                # Extract x and y coordinates from the received message
        x = msg.pose.position.x
        y = msg.pose.position.y
        if self.is_within_threshold(x, y):         # If within a certain threshold, terminate the process
            return
        self.coordinates.append((x, y))              # Append coordinates to the list and print the length of the list
        print("Count : ", len(self.coordinates))

    def is_within_threshold(self, x, y):     # Function to check if the coordinates are within a certain threshold
        for coord in self.coordinates:
            if self.calculate_distance(coord[0], coord[1], x, y) < 0.081:             # Check against a certain distance from previous coordinates
                return True
        return False
    # Function to calculate the Euclidean distance between two points
    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()