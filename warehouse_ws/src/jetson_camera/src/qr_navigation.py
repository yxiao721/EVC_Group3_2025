#!/usr/bin/env python2
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetson_camera.msg import ImagePair
from jetson_camera.msg import QRdata
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import threading
from pyzbar import pyzbar
import time
import os
import signal
import numpy as np
import sys

class SimpleQRNode:
    def __init__(self):
        rospy.init_node('simple_qr_node', anonymous=True)
        self.bridge = CvBridge()

        # Simple parameters
        self.target_distance = 20  # Goal distance in cm
        self.target_width_px = 160  # QR code width in pixels at target distance
        self.distance_tolerance = 5  # Tolerance in cm
        self.centering_tolerance = 0.15  # 8% of image width (tighter)
        
        # Control parameters - smaller steps for precision
        self.move_step = 1  # Move 3cm at a time (smaller steps)
        self.rotate_step = 1  # Rotate 5 degrees at a time (smaller steps)
        
        # QR tracking
        self.qr_center_x = 0
        self.current_distance = 0
        self.image_center_x = 0
        self.qr_data = 0  # Store QR code as integer (changed from empty string)
        self.task_published = False  # Flag to prevent multiple task publications
        
        # Timing
        self.last_command_time = rospy.Time.now()
        self.command_delay = rospy.Duration(1.0)  # 1 second between commands
        
        # Publishers
        self.task_pub = rospy.Publisher('/qr_task', QRdata, queue_size=1)  # Task publisher
        self.barcode_pub = rospy.Publisher('/barcode_number', Int32, queue_size=1)  # Changed to publish actual number
        
        # Display
        cv2.namedWindow("Simple QR", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Simple QR", 640, 480)
        
        # Threading
        signal.signal(signal.SIGINT, self.signal_handler)
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.is_running = True
        
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()
        
        # Camera subscriber
        self.sub = rospy.Subscriber(
            "/camera/processed_image",
            ImagePair,
            self.callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("Simple QR node started. Target distance: %dcm", self.target_distance)
    
    def signal_handler(self, sig, frame):
        self.cleanup()
        sys.exit(0)

    def estimate_distance(self, qr_width_px, qr_height_px):
        """Improved distance estimation using both width and height"""
        if qr_width_px == 0 or qr_height_px == 0:
            return float('inf')
        
        # Use average of width and height for more stable distance
        avg_size = (qr_width_px + qr_height_px) / 2.0
        return (self.target_width_px / avg_size) * self.target_distance

    def get_qr_center_with_tilt_compensation(self, qr_polygon):
        """Get QR center accounting for perspective/tilt"""
        if len(qr_polygon) == 4:
            # Calculate center from all 4 corners for better accuracy
            points = [(point.x, point.y) for point in qr_polygon]
            center_x = sum(p[0] for p in points) / 4.0
            center_y = sum(p[1] for p in points) / 4.0
            return int(center_x), int(center_y)
        else:
            # Fallback to bounding box center
            return None, None

    def process_qr(self, img):
        """Simple QR processing - just rotate and move"""
        current_time = rospy.Time.now()
        
        # Don't send commands too quickly
        if current_time - self.last_command_time < self.command_delay:
            return img
        
        # Get image dimensions
        height, width = img.shape[:2]
        self.image_center_x = width // 2
        
        # Detect QR codes
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        qr_codes = pyzbar.decode(gray)

        if qr_codes:
            largest_qr = max(qr_codes, key=lambda q: q.rect.width * q.rect.height)
            (x, y, w, h) = largest_qr.rect
            raw_qr_data = largest_qr.data.decode('utf-8')
            rospy.loginfo("Raw QR data: %s", raw_qr_data)
            
            # Convert QR data to integer - try multiple formats
            qr_number = None
            if raw_qr_data.startswith("qr") and raw_qr_data[2:].isdigit():
                qr_number = int(raw_qr_data[2:])
                rospy.loginfo("Converted QR number from 'qr' format: %d", qr_number)
            elif raw_qr_data.isdigit():
                qr_number = int(raw_qr_data)
                rospy.loginfo("Converted QR number from digit format: %d", qr_number)
            else:
                # If we can't convert to integer, use a default
                rospy.logwarn("Cannot convert QR data '%s' to integer, using default ID 999", raw_qr_data)
                qr_number = 999
            
            # Always store as integer for QRdata messages
            self.qr_data = qr_number

            # Publish barcode number for task manager (only for numbers > 10 to avoid navigation QR codes)
            if qr_number > 10:
                last_digit = qr_number % 10
                rospy.loginfo("Publishing barcode number: %d (ends with %d)", qr_number, last_digit)
                self.barcode_pub.publish(Int32(data=qr_number))

            # Get better center position accounting for tilt
            tilt_center_x, tilt_center_y = self.get_qr_center_with_tilt_compensation(largest_qr.polygon)
            
            if tilt_center_x is not None:
                self.qr_center_x = tilt_center_x
                qr_center_y = tilt_center_y
            else:
                # Fallback to bounding box center
                self.qr_center_x = x + w // 2
                qr_center_y = y + h // 2
            
            # Improved distance estimation
            self.current_distance = self.estimate_distance(w, h)
            
            # Draw QR detection with polygon for tilt visualization
            if len(largest_qr.polygon) == 4:
                points = np.array([(point.x, point.y) for point in largest_qr.polygon], np.int32)
                cv2.polylines(img, [points], True, (0, 255, 0), 2)
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw centers
            cv2.circle(img, (self.qr_center_x, qr_center_y), 3, (255, 0, 0), -1)
            cv2.line(img, (self.image_center_x, 0), (self.image_center_x, height), (0, 0, 255), 1)
            
            # Display info
            cv2.putText(img, "Dist: {:.1f}cm".format(self.current_distance),
                       (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(img, "Target: {}cm".format(self.target_distance),
                       (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(img, "QR: {}".format(self.qr_data),
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "Raw: {}".format(raw_qr_data),
                       (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(img, "Size: {}x{}".format(w, h),
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # Calculate offsets
            x_offset = self.qr_center_x - self.image_center_x
            normalized_offset = x_offset / float(self.image_center_x)
            distance_error = self.current_distance - self.target_distance
            
            # Simple control logic with smaller, more precise movements
            if abs(normalized_offset) > self.centering_tolerance:
                # Reset task published flag when not in perfect position
                self.task_published = False
                
                # Need to rotate to center QR code
                if normalized_offset > 0:
                    # QR is to the right, rotate right
                    print("MOVING: QR to the right | Distance: {:.1f}cm | Angle offset: {:.2f} | Rotating RIGHT {} degrees".format(
                        self.current_distance, normalized_offset, self.rotate_step))
                    rospy.loginfo("QR to the right (offset: %.2f), rotating right %d degrees", 
                                normalized_offset, self.rotate_step)
                    output = QRdata()
                    output.qr_id = self.qr_data
                    output.move = 0
                    output.rotate = -self.rotate_step
                    output.perfect = 0
                    self.task_pub.publish(output)
                    cv2.putText(img, "Rotating RIGHT", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                else:
                    # QR is to the left, rotate left
                    print("MOVING: QR to the left | Distance: {:.1f}cm | Angle offset: {:.2f} | Rotating LEFT {} degrees".format(
                        self.current_distance, normalized_offset, self.rotate_step))
                    rospy.loginfo("QR to the left (offset: %.2f), rotating left %d degrees", 
                                normalized_offset, self.rotate_step)
                    output = QRdata()
                    output.qr_id = self.qr_data
                    output.move = 0
                    output.rotate = self.rotate_step
                    output.perfect = 0
                    self.task_pub.publish(output)
                    cv2.putText(img, "Rotating LEFT", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                self.last_command_time = current_time
                
            elif abs(distance_error) > self.distance_tolerance:
                # Reset task published flag when not in perfect position
                self.task_published = False
                
                # QR is centered, now adjust distance
                if distance_error > 0:
                    # Too far, move forward
                    print("MOVING: Too far | Current distance: {:.1f}cm | Target: {}cm | Error: {:.1f}cm | Moving FORWARD {}cm".format(
                        self.current_distance, self.target_distance, distance_error, self.move_step))
                    rospy.loginfo("Too far (%.1fcm), moving forward %dcm", self.current_distance, self.move_step)
                    output = QRdata()
                    output.qr_id = self.qr_data
                    output.move = self.move_step
                    output.rotate = 0
                    output.perfect = 0
                    self.task_pub.publish(output)
                    cv2.putText(img, "Moving FORWARD", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                else:
                    # Too close, move backward
                    print("MOVING: Too close | Current distance: {:.1f}cm | Target: {}cm | Error: {:.1f}cm | Moving BACKWARD {}cm".format(
                        self.current_distance, self.target_distance, abs(distance_error), self.move_step))
                    rospy.loginfo("Too close (%.1fcm), moving backward %dcm", self.current_distance, self.move_step)
                    output = QRdata()
                    output.qr_id = self.qr_data
                    output.move = -self.move_step
                    output.rotate = 0
                    output.perfect = 0
                    self.task_pub.publish(output)
                    cv2.putText(img, "Moving BACKWARD", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                self.last_command_time = current_time
                
            else:
                # Perfect position!
                print("PERFECT POSITION! | Distance: {:.1f}cm | Angle offset: {:.2f} | Target: {}cm | QR: {}".format(
                    self.current_distance, normalized_offset, self.target_distance, self.qr_data))
                rospy.loginfo("Perfect position! Distance: %.1fcm, Offset: %.2f, QR: %s", 
                            self.current_distance, normalized_offset, self.qr_data)
                cv2.putText(img, "PERFECT POSITION!", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish perfect alignment message
                output = QRdata()
                output.qr_id = self.qr_data
                output.move = 0
                output.rotate = 0
                output.perfect = 1
                self.task_pub.publish(output)
                
        else:
            # No QR code found - reset task published flag
            self.task_published = False
            self.qr_data = 0  # Set to 0 instead of empty string to avoid type errors
            cv2.putText(img, "Searching for QR code...", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return img

    def callback(self, msg):
        """Camera callback"""
        try:
            undistorted_img = self.bridge.imgmsg_to_cv2(msg.undistorted_image, "bgr8")
            processed_img = self.process_qr(undistorted_img)
            
            with self.image_lock:
                self.latest_image = processed_img

        except Exception as e:
            rospy.logwarn("Error processing frame: %s", str(e))

    def display_loop(self):
        """Display camera feed"""
        rate = rospy.Rate(15)
        
        while self.is_running and not rospy.is_shutdown():
            with self.image_lock:
                if self.latest_image is not None:
                    cv2.imshow("Simple QR", self.latest_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("Quit requested")
                self.cleanup()
                break
                
            rate.sleep()

    def cleanup(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down simple QR node...")
        self.is_running = False
        if hasattr(self, 'display_thread') and self.display_thread.is_alive():
            self.display_thread.join(1.0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    node = SimpleQRNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rospy.loginfo("Simple QR node shut down successfully")