#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import csv
import os
from datetime import datetime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
import math

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        
        self.bridge = CvBridge()
        self.linear_speed = 0.08
        self.max_angular_speed = 0.5
        
        # PID parameters
        self.Kp = 0.003
        self.Ki = 0.00001
        self.Kd = 0.005
        
        self.error_sum = 0
        self.last_error = 0
        
        self.log_data = []
        self.current_pose = None
        self.start_time = None
        self.image_received = False
        
        # Lap completion
        self.start_x = None
        self.start_y = None
        self.distance_traveled = 0.0
        self.last_x = None
        self.last_y = None
        self.lap_complete = False
        
        self.robot_path = Path()
        self.robot_path.header.frame_id = "odom"
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.processed_img_pub = rospy.Publisher('/line_detection/image', Image, queue_size=1)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("LINE FOLLOWER STARTED")
        rospy.loginfo("Robot will complete ONE LAP and STOP")
        rospy.loginfo("=" * 60)
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Record start position
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.last_x = x
            self.last_y = y
            rospy.loginfo(f"Start position: ({x:.3f}, {y:.3f})")
            return
        
        # Calculate distance traveled
        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
            self.last_x = x
            self.last_y = y
        
        # Check if back at start (after traveling enough distance)
        if not self.lap_complete and self.distance_traveled > 6.0:  # Pentagon perimeter ~7m
            dist_to_start = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
            
            if dist_to_start < 0.25:  # Within 25cm of start
                self.lap_complete = True
                
                # STOP THE ROBOT
                stop = Twist()
                stop.linear.x = 0.0
                stop.angular.z = 0.0
                self.cmd_vel_pub.publish(stop)
                
                rospy.loginfo("=" * 60)
                rospy.loginfo("✓ LAP COMPLETED!")
                rospy.loginfo(f"Total distance: {self.distance_traveled:.2f}m")
                rospy.loginfo(f"Final position: ({x:.3f}, {y:.3f})")
                rospy.loginfo(f"Distance to start: {dist_to_start:.3f}m")
                rospy.loginfo("=" * 60)
                
                self.save_logs()
                rospy.sleep(1.0)
                rospy.signal_shutdown("Lap completed successfully!")
        
        self.current_pose = msg.pose.pose
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.robot_path.poses.append(pose_stamped)
        
        if len(self.robot_path.poses) % 10 == 0:
            self.robot_path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.robot_path)
    
    def image_callback(self, msg):
        # Stop processing if lap complete
        if self.lap_complete:
            return
            
        if not self.image_received:
            rospy.loginfo("✓ Camera connected!")
            self.image_received = True
            self.start_time = rospy.Time.now()
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        try:
            processed_img, error, line_detected, lateral_error_m = self.detect_line(cv_image)
            
            try:
                self.processed_img_pub.publish(self.bridge.cv2_to_imgmsg(processed_img, "bgr8"))
            except:
                pass
            
            if line_detected:
                twist = self.calculate_control(error)
                self.cmd_vel_pub.publish(twist)
                self.log_performance(error, lateral_error_m)
            else:
                twist = Twist()
                twist.linear.x = 0.03
                twist.angular.z = 0.25
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn_throttle(3.0, "⚠ Line not detected - searching...")
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")
    
    def detect_line(self, image):
        h, w = image.shape[:2]
        
        roi_start = int(h * 0.5)
        roi = image[roi_start:h, :]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        
        _, binary = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((9, 9), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=3)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        vis = image.copy()
        cv2.rectangle(vis, (0, roi_start), (w, h), (0, 255, 0), 2)
        
        line_detected = False
        error = 0
        lateral_error_m = 0
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > 1000:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cv2.drawContours(vis[roi_start:h], [largest], -1, (0, 255, 0), 3)
                    cv2.circle(vis, (cx, roi_start + cy), 15, (0, 0, 255), -1)
                    
                    center = w // 2
                    error = cx - center
                    
                    pixels_per_meter = w / 0.5
                    lateral_error_m = error / pixels_per_meter
                    
                    cv2.line(vis, (center, 0), (center, h), (255, 0, 0), 2)
                    cv2.line(vis, (center, roi_start + cy), (cx, roi_start + cy), (0, 255, 255), 3)
                    
                    cv2.putText(vis, f"Error: {error}px ({lateral_error_m:.3f}m)", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(vis, f"Distance: {self.distance_traveled:.2f}m", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    line_detected = True
                    rospy.loginfo_throttle(2.0, f"Line | Dist: {self.distance_traveled:.2f}m | Error: {lateral_error_m:.3f}m")
        
        status = "FOLLOWING" if line_detected else "SEARCHING"
        color = (0, 255, 0) if line_detected else (0, 0, 255)
        cv2.putText(vis, status, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
        
        return vis, error, line_detected, lateral_error_m
    
    def calculate_control(self, error):
        self.error_sum += error
        self.error_sum = max(-2000, min(2000, self.error_sum))
        
        error_diff = error - self.last_error
        
        angular_z = -(self.Kp * error + self.Ki * self.error_sum + self.Kd * error_diff)
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        self.last_error = error
        
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z
        
        return twist
    
    def log_performance(self, error_px, error_m):
        if self.current_pose is not None:
            timestamp = rospy.Time.now().to_sec()
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            z = self.current_pose.position.z
            
            orientation_q = self.current_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            
            self.log_data.append({
                'timestamp': timestamp,
                'time_elapsed': timestamp - self.start_time.to_sec() if self.start_time else 0,
                'x': x, 'y': y, 'z': z,
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'lateral_error_pixels': error_px,
                'lateral_error_m': error_m,
                'distance_traveled': self.distance_traveled
            })
    
    def save_logs(self):
        if not self.log_data:
            return
        
        package_path = os.path.expanduser('~/catkin_ws/src/turtlebot3_line_follower')
        log_dir = os.path.join(package_path, 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(log_dir, f'log_{timestamp}.csv')
        stats_file = os.path.join(log_dir, f'stats_{timestamp}.txt')
        
        with open(csv_file, 'w', newline='') as f:
            fields = ['timestamp', 'time_elapsed', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                     'lateral_error_pixels', 'lateral_error_m', 'distance_traveled']
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for data in self.log_data:
                writer.writerow(data)
        
        errors = [abs(d['lateral_error_m']) for d in self.log_data]
        total_time = self.log_data[-1]['time_elapsed'] if self.log_data else 0
        
        with open(stats_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("PENTAGON LINE FOLLOWER - PERFORMANCE STATISTICS\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Samples: {len(self.log_data)}\n")
            f.write(f"Time: {total_time:.2f} seconds\n")
            f.write(f"Distance: {self.distance_traveled:.3f} meters\n\n")
            f.write("LATERAL ERROR:\n")
            f.write(f"  Mean: {np.mean(errors):.4f} m\n")
            f.write(f"  Max: {np.max(errors):.4f} m\n")
            f.write(f"  Min: {np.min(errors):.4f} m\n")
            f.write(f"  Within 0.1m: {sum(1 for e in errors if e <= 0.1)/len(errors)*100:.1f}%\n")
            f.write("=" * 60 + "\n")
        
        rospy.loginfo(f"✓ Logs saved: {csv_file}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'follower' in locals():
            follower.save_logs()
