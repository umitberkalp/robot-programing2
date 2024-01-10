import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/limo/object_location',
            self.posestamped_callback,
            10
        )
        self.coordinates = []
    def posestamped_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        if self.is_within_threshold(x, y):
            return
        self.coordinates.append((x, y))
        print("Count : ", len(self.coordinates))

    def is_within_threshold(self, x, y):
        for coord in self.coordinates:
            if self.calculate_distance(coord[0], coord[1], x, y) < 0.077:
                return True
        return False

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