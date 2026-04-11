import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# The magic QoS import that fixes the video stream!
from rclpy.qos import qos_profile_sensor_data

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # 1. The Listener: Matches Gazebo's livestream
        self.subscription = self.create_subscription(
            Image, 
            '/conveyor_camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data)
            
        # 2. The Broadcasters: Talking to the Conductor and Velocity Tracker
        self.vel_publisher = self.create_publisher(TwistStamped, '/target/smoothed_velocity', 10)
        self.pos_publisher = self.create_publisher(Point, '/box_position', 10)
        
        self.bridge = CvBridge()

        # --- Tracking Variables ---
        self.last_pos = None
        self.last_time = None
        self.smoothed_velocity = np.array([0.0, 0.0, 0.0])
        self.alpha = 0.2 

        # We use a specific scale factor to map the 640x480 pixels to Gazebo meters
        self.scale_factor = 0.001966 

    def image_callback(self, msg):
        self.get_logger().info("Camera Connected! Tracking BLUE objects...", once=True)
        
        # Convert ROS video feed into OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- THE COLOR FIX: Track Blue instead of White ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110, 150, 0])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # ==========================================
        # THE ROI MASK (THE BLINDER)
        # ==========================================
        # Draw a solid black rectangle over the left side to hide the UR5e base!
        cv2.rectangle(mask, (0, 0), (120, 480), (0, 0, 0), -1)

        # Find shapes in the filtered image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest shape on the screen
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Size limit: Ignore tiny noise or massive glitches
            area = cv2.contourArea(largest_contour)
            if 50 < area < 3000: 
                
                # Find the mathematical center of the box
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw a bright green dot on the center of the box!
                    cv2.circle(cv_image, (cx, cy), 8, (0, 255, 0), -1)

                    # --- Convert to proper Gazebo World Coordinates ---
                    world_x = -(cy - 240) * self.scale_factor
                    world_y = -(cx - 320) * self.scale_factor

                    # 1. Broadcast the exact coordinates to the Conductor
                    target_msg = Point()
                    target_msg.x = world_x
                    target_msg.y = world_y
                    target_msg.z = 0.0
                    self.pos_publisher.publish(target_msg)

                    # 2. Velocity Tracking & Smoothing Math
                    pos = np.array([world_x, world_y, 0.0])
                    current_time = time.time()
                    if self.last_pos is not None and self.last_time is not None:
                        dt = current_time - self.last_time
                        if dt > 0:
                            raw_velocity = (pos - self.last_pos) / dt
                            self.smoothed_velocity = (self.alpha * raw_velocity) + ((1.0 - self.alpha) * self.smoothed_velocity)
                            
                    self.last_pos = pos
                    self.last_time = current_time

        # Show the normal video window with the green dot
        cv2.imshow("Filtered Tracking", cv_image)
        # Show the computer's brain (You will see the black rectangle blocking the robot!)
        cv2.imshow("Computer Vision Debug", mask) 
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()