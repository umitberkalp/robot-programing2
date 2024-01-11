import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover(Node):
    """
    A very simple Roamer implementation for LIMO.
    It simply goes straight until any obstacle is within
    turns right.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('tf_listener')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
   
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        min_dist = min(data.ranges[int(len(data.ranges)/2) -30 : int(len(data.ranges)/2) +30])         # Extract the minimum distance from the center 60 degrees of the laser scan data
        
        t = Twist()
        if min_dist < 0.46: # Check if there is an obstacle within a certain distance
            t.angular.z = -0.5             # If obstacle is detected, turn right
        else:
            t.linear.x = 0.2                # If no obstacle, go straight
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    