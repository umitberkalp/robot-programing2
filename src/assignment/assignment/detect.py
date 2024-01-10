import rclpy
from rclpy.node import Node
from rclpy import qos
import numpy as np
import cv2
from cv2 import waitKey
from tf2_ros import Buffer, TransformListener
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker

font = cv2.FONT_HERSHEY_SIMPLEX

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None
    visualisation = True
    color2depth_aspect = 1.0 

    def __init__(self):    
        super().__init__('image_projection_3')
        self.bridge = CvBridge()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                                self.camera_info_callback,
                                                qos_profile=qos.qos_profile_sensor_data)
       
        self.object_location_pub = self.create_publisher(PoseStamped, '/limo/object_location', 10)

        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw',
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
       
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw',
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.coordinates = []

    def get_tf_transform(self, target_frame, source_frame):
        try:             # Look up the transform between target_frame and source_frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:             # Initialize camera model from camera info
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        if self.camera_model is None:
            return
        if self.image_depth_ros is None:
            return
        try:             # Convert color and depth images from ROS messages to OpenCV format
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)         # Convert color image to HSV space
        lower_pink = np.array([150, 50, 50])# Define color range for pink in HSV space
        upper_pink = np.array([170, 255, 255])
        image_mask = cv2.inRange(hsv, lower_pink, upper_pink)         # Create a mask for pink color
                # Find contours of pink objects in the mask
        contours, _, = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] == 0:
                return
            #comment
            image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])             # Calculate image coordinates of the object
            cv2.drawContours(image_color, [contour], -1, (0, 255, 0), 2)
                        # Calculate depth coordinates using color image coordinates
            depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect,
                image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]             # Obtain depth value at the calculated depth coordinates
            camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))  # Project image coordinates to 3D camera coordinates
            camera_coords = [x/camera_coords[2] for x in camera_coords]
            camera_coords = [x*depth_value for x in camera_coords]

            object_location = PoseStamped() # Create PoseStamped message with object location information
            object_location.header.frame_id = "depth_link"
            object_location.pose.orientation.w = 0.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]
            object_location.pose.orientation.w = 0.0
            self.object_location_pub.publish(object_location)  

        #print("coords : ", camera_coords)
        cv2.imshow("image color", image_color)
        cv2.waitKey(1)
           
def main(args=None):
    rclpy.init(args=args)
    image_projection = ObjectDetector()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

