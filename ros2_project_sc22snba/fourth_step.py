# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import signal
import math
import asyncio

def euler_to_quaternion(yaw, pitch=0, roll=0):
    """
    Convert Euler angles to quaternion values
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w

    return q


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Navigation and exploration setup
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define exploration waypoints (x, y, theta) adjusted for map origin
        # Each grid square is 1x1m, coordinates adjusted for map origin
        self.waypoints = [
            (-10.2, -13.8, 0.0),      # Bottom right compartment
            (-10.2, -10.8, math.pi/2), # Middle right area
            (-10.2, -7.8, math.pi/2),  # Top right compartment
            (-7.2, -7.8, math.pi),     # Top middle area
            (-4.2, -7.8, math.pi),     # Top left compartment
            (-4.2, -10.8, -math.pi/2), # Middle left area
            (-4.2, -13.8, -math.pi/2), # Bottom left compartment
            (-7.2, -13.8, 0.0),        # Bottom middle area
            (-7.2, -10.8, 0.0),        # Center of the map
        ]
        
        # Rest of initialization
        self.current_waypoint = 0
        self.exploring = False
        
        # Existing initialization code
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        self.green_found = False
        self.red_found = False
        self.blue_found = False
        self.blue_contour_area = 0
        
        # Calibrated values for 1-meter distance based on apparent box size
        self.target_area = 20000     # Increased target area for 1m distance
        self.area_tolerance = 5000    # Wider tolerance for more stable positioning
        self.stable_count = 0
        self.stable_threshold = 3     # Need fewer stable readings to stop
        self.is_stable = False
        self.max_speed = 0.15        # Reduced max speed for more controlled movement
        
        # Angular control parameters
        self.center_x = None
        self.image_center_x = 320
        self.center_tolerance = 50    # Increased tolerance for easier centering
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)
        self.too_close = False



    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Set HSV bounds for all three colors
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255])

        # Convert to HSV
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter colors
        green_image = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        red_image = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        # Combine all color masks
        mask_image = cv2.bitwise_or(cv2.bitwise_or(green_image, red_image), blue_image)
        final_image = cv2.bitwise_and(image, image, mask=mask_image)

        # Reset detection flags
        self.green_found = False
        self.red_found = False
        self.blue_found = False
        
        # Process green contours
        green_contours, _ = cv2.findContours(green_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) > 0:
            c = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (0,255,0), 2)  # Green circle
                self.green_found = True
                print("Green detected")

        # Process red contours
        red_contours, _ = cv2.findContours(red_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(red_contours) > 0:
            c = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (0,0,255), 2)  # Red circle
                self.red_found = True
                print("Red detected")

        # Process blue contours
        blue_contours, _ = cv2.findContours(blue_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(blue_contours) > 0:
            c = max(blue_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (255,0,0), 2)  # Blue circle
                self.blue_found = True
                self.blue_contour_area = cv2.contourArea(c)
                self.center_x = center[0]  # Store x-coordinate of blue box center
                print(f"Blue detected at x={self.center_x}, area={self.blue_contour_area}")

        # Show only the processed image with detections
        cv2.namedWindow('Color Detection',cv2.WINDOW_NORMAL)
        cv2.imshow('Color Detection', image)
        cv2.resizeWindow('Color Detection',320,240)
        cv2.waitKey(3)

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Forward with 0.2 m/s

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Backward with 0.2 m/s
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)

    def create_pose_stamped(self, x, y, theta):
        """Create a PoseStamped message from x, y coordinates and theta orientation"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = euler_to_quaternion(theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
        
    def navigate_to_waypoint(self):
        """Navigate to the current waypoint"""
        if self.current_waypoint >= len(self.waypoints):
            self.current_waypoint = 0
            
        x, y, theta = self.waypoints[self.current_waypoint]
        pose = self.create_pose_stamped(x, y, theta)
        
        # Send navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        self.nav_client.wait_for_server()
        
        # Send the goal
        send_goal_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted')
        
        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result()
        
        if result.status != 4:  # 4 is SUCCESS in action msgs
            self.get_logger().error(f'Goal failed with status: {result.status}')
            return False
            
        self.get_logger().info('Goal succeeded!')
        return True

    def maintain_blue_distance(self):
        """Control the robot to maintain ~1m distance from blue box"""
        if not self.blue_found or self.center_x is None:
            self.stable_count = 0
            self.is_stable = False
            return False
            
        desired_velocity = Twist()
        
        # First, ensure we're centered on the blue box
        x_offset = self.center_x - self.image_center_x
        is_centered = abs(x_offset) < self.center_tolerance
        
        if not is_centered:
            # Only rotate to center if significantly off
            desired_velocity.angular.z = 0.2 if x_offset < 0 else -0.2
            desired_velocity.linear.x = 0.0  # Don't move forward/back while rotating
            print(f"Centering on blue box, offset: {x_offset}")
            self.stable_count = 0
        else:
            desired_velocity.angular.z = 0.0
            
            # Check if we're at the right distance
            if self.is_stable:
                # If we're already stable, maintain position
                desired_velocity.linear.x = 0.0
                print("Holding position at 1m from blue box")
            else:
                distance_error = self.blue_contour_area - self.target_area
                in_target_range = abs(distance_error) < self.area_tolerance
                
                if in_target_range:
                    self.stable_count += 1
                    if self.stable_count >= self.stable_threshold:
                        print("Position stabilized at 1m from blue box - STOPPING")
                        self.is_stable = True
                        desired_velocity.linear.x = 0.0
                    else:
                        # Slow to a stop while stabilizing
                        desired_velocity.linear.x = 0.0
                        print(f"Almost there, stabilizing... {self.stable_count}/{self.stable_threshold}")
                else:
                    # Reset stability counter
                    self.stable_count = 0
                    
                    # Determine direction and speed
                    if self.blue_contour_area > self.target_area:
                        # Too close, back up
                        desired_velocity.linear.x = -self.max_speed
                        print(f"Too close (area={self.blue_contour_area}), backing up")
                    else:
                        # Too far, move forward
                        desired_velocity.linear.x = self.max_speed
                        print(f"Too far (area={self.blue_contour_area}), moving closer")
        
        self.publisher.publish(desired_velocity)
        return True


# Create a node of the class in the main and ensure it stays up and running
def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start exploration immediately
        robot.exploring = True
        
        while rclpy.ok():
            if robot.blue_found:
                # When blue is found, maintain distance and stop exploring
                robot.exploring = False
                robot.maintain_blue_distance()
            elif robot.exploring:
                # Navigate to next waypoint if we're exploring
                success = robot.navigate_to_waypoint()
                if success:
                    robot.current_waypoint += 1
                    print(f"Moving to waypoint {robot.current_waypoint}")
            
            rclpy.spin_once(robot, timeout_sec=0.1)
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cv2.destroyAllWindows()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
