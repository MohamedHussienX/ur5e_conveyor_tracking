import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Twist 
import time
import threading
import math 

class ConductorNode(Node):
    def __init__(self):
        super().__init__('conductor_node')
        
        # =========================================================
        # 📍 EDITABLE DROP-OFF LOCATION
        # =========================================================
        self.drop_x = 0.3    
        self.drop_y = 0.3    
        self.drop_z = 1.3    
        # =========================================================

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.box_pub = self.create_publisher(Twist, '/box_cmd_vel', 10)

        self.get_logger().info("Waiting for Arm Controller...")
        self.arm_client.wait_for_server()
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            pass

        self.is_busy = False
        self.last_time = None
        self.last_x = None
        self.last_y = None
        
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.frame_count = 0 

        self.camera_sub = self.create_subscription(Point, '/box_position', self.box_callback, 10)
        self.get_logger().info("All Systems Go! Waiting for targets...")

    def box_callback(self, msg):
        current_time = time.time()
        
        if self.last_time is not None and self.last_x is not None and self.last_y is not None:
            dt = current_time - self.last_time
            if dt > 0:
                dx = msg.x - self.last_x
                dy = msg.y - self.last_y
                
                raw_vx = dx / dt
                raw_vy = dy / dt
                
                self.vx = (0.2 * raw_vx) + (0.8 * self.vx)
                self.vy = (0.2 * raw_vy) + (0.8 * self.vy)
                
                self.speed = math.sqrt(self.vx**2 + self.vy**2)
                
        self.last_time = current_time
        self.last_x = msg.x
        self.last_y = msg.y
        
        if not self.is_busy:
            self.frame_count += 1
            
            if self.frame_count > 30: 
                self.is_busy = True 
                
                if self.speed > 0.01:
                    predicted_x = msg.x + (self.vx * 6.0)
                    catch_x = max(-0.5, min(0.5, predicted_x))
                    catch_y = msg.y 
                    
                    distance_to_catch = abs(catch_x - msg.x)
                    time_to_arrival = distance_to_catch / abs(self.vx) if abs(self.vx) > 0.001 else 0.0
                    
                    self.get_logger().info(f"*** MOVING TARGET DETECTED! Speed: {self.speed:.3f} m/s ***")
                    self.get_logger().info(f"*** Setting Ambush at X={catch_x:.2f}. Arrival in {time_to_arrival:.2f}s ***")
                else:
                    catch_x = msg.x
                    catch_y = msg.y
                    time_to_arrival = 0.0 
                    self.get_logger().info(f"*** STATIONARY BOX DETECTED! Reaching directly for X={catch_x:.2f}, Y={catch_y:.2f} ***")

                thread = threading.Thread(target=self.run_sequence, args=(catch_x, catch_y, time_to_arrival))
                thread.start()

    def run_sequence(self, catch_x, catch_y, time_to_arrival):
        start_time = time.time()
        self.move_gripper(0.0) 

        safe_x = max(-0.5, min(0.5, catch_x))
        safe_y = max(-0.25, min(0.25, catch_y))

        self.get_logger().info(f"Moving to Hover position (Safe X={safe_x:.2f}, Y={safe_y:.2f})...")
        
        # 1. Move to Ambush Hover
        hover_joints = self.get_ik(x=safe_x, y=safe_y, z=1.05)
        if not hover_joints:
             self.get_logger().error(f"IK Failed for Hover! Aborting.")
             self.is_busy = False
             return
             
        self.move_arm(hover_joints, duration=2.0)
        time.sleep(2.5) 

        elapsed = time.time() - start_time
        dive_time = 1.0 
        latency_offset = 0.25 
        time_left = (time_to_arrival - elapsed) - (dive_time + latency_offset)
        
        if time_left > 0:
            self.get_logger().info(f"Arm in position. Waiting {time_left:.2f}s in the blind spot...")
            time.sleep(time_left)

        self.get_logger().info("BOX IS IN THE ZONE! STRIKE!")
        
        # 2. Dive straight down to bracket the box
        grasp_joints = self.get_ik(x=safe_x, y=safe_y, z=0.885) 
        
        if grasp_joints:
            self.move_arm(grasp_joints, duration=dive_time)
            time.sleep(dive_time + 0.1) 

            # 3. Clamp tightly into the mechanical groove
            self.get_logger().info("Clamping into the mechanical lock...")
            self.move_gripper(0.35) 
            time.sleep(1.0) 

            # 4. Hit the brakes! Stop the box
            stop_msg = Twist()
            self.box_pub.publish(stop_msg)
            
            # 5. Lift the payload
            self.get_logger().info("Lifting Payload...")
            self.move_arm(hover_joints, duration=3.0)
            time.sleep(3.2)
            
            # 6. Move to the Custom Drop Zone
            self.get_logger().info(f"Moving to Drop Zone: X={self.drop_x}, Y={self.drop_y}, Z={self.drop_z}")
            drop_joints = self.get_ik(x=self.drop_x, y=self.drop_y, z=self.drop_z)
            
            if drop_joints:
                self.move_arm(drop_joints, duration=2.5)
                time.sleep(3.0)
                
                # 7. Release the box
                self.get_logger().info("Releasing Payload.")
                self.move_gripper(0.0) 
                time.sleep(1.0)
            else:
                self.get_logger().error("IK MATH FAILED for Drop Zone! The coordinates might be out of the robot's physical reach.")
            
            # 8. Return to safe Hover
            self.get_logger().info("Resetting Arm...")
            self.move_arm(hover_joints, duration=2.0)
            time.sleep(2.5)
            
        else:
            self.get_logger().error(f"IK MATH FAILED! Cannot physically bend to reach X={safe_x:.2f}, Y={safe_y:.2f}, Z=0.885")
            time.sleep(3.0)
            
        self.get_logger().info("Sequence Complete. Ready for next target...")
        
        # Reset trackers
        self.speed = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.frame_count = 0 
        self.is_busy = False 
        
    def get_ik(self, x, y, z):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        # Keep the gripper pointing straight down
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 1.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 0.0

        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()

        if response.error_code.val == 1:
            arm_joints = []
            arm_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            for name in arm_names:
                idx = response.solution.joint_state.name.index(name)
                arm_joints.append(response.solution.joint_state.position[idx])
            return arm_joints
        else:
            return None

    def move_arm(self, target_joints, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(point)
        self.arm_client.send_goal_async(goal_msg)

    def move_gripper(self, target_position):
        msg = Float64MultiArray()
        x = target_position
        msg.data = [x, -x, x, -x, -x, x] 
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConductorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()