#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from enum import Enum
from std_msgs.msg import Int32, Bool, String
from jetson_camera.msg import QRdata

class RobotState(Enum):
    PARKING = 1      # Trying to align with QR code
    PARKED = 2       # Successfully parked, waiting for face/barcode detection
    TASK_SENT = 3    # Task published, waiting for automatic movement to complete

class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager", anonymous=True)
        
        # State management
        self.state = RobotState.PARKING
        self.pending_face_detection = None  # Store face detection with higher priority
        self.pending_barcode_number = None  # Store barcode number for task decision
        self.parked_timer = None  # Store timer reference
        self.task_sent_timer = None  # Timer to return to parking after task sent
        self.last_perfect_time = rospy.Time(0)  # Track when we last saw perfect alignment
        self.task_executed = False  # Flag to prevent duplicate task execution
        
        # Publishers
        self.move_pub = rospy.Publisher('/move', Int32, queue_size=1)
        self.rotate_pub = rospy.Publisher('/rotate', Int32, queue_size=1)
        self.task_pub = rospy.Publisher('/task', Int32, queue_size=1)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/qr_task', QRdata, self.qr_callback)
        rospy.Subscriber('/face_detection', Int32, self.face_callback)  # Only Int32, not String
        rospy.Subscriber('/barcode_number', Int32, self.barcode_callback)  # Barcode number
        
        rospy.loginfo("Task Manager started - will park at ANY QR code")

    def handle_navigation_feedback(self, qr_msg):
        """Handle QR code alignment feedback"""
        if qr_msg.perfect == 1:
            return True  # Perfect alignment achieved
        elif qr_msg.move != 0:
            self.move_pub.publish(qr_msg.move)
        elif qr_msg.rotate != 0:
            self.rotate_pub.publish(qr_msg.rotate)
        return False  # Still aligning

    def select_and_execute_task(self):
        """Single decision point for task selection - only executes for specific conditions"""
        
        # Prevent duplicate execution
        if self.task_executed:
            return
            
        task_to_execute = None
        reason = ""
        
        # Obama face detection - Task 1
        if self.pending_face_detection == "Obama":
            task_to_execute = 1
            reason = "Obama detected"
            rospy.loginfo("OBAMA DETECTED - AUTHORIZED ACCESS")
            
        # Barcode ending in 7 - Task 2
        elif self.pending_barcode_number is not None:
            barcode_number = self.pending_barcode_number
            last_digit = barcode_number % 10
            
            if last_digit == 7:
                task_to_execute = 2
                reason = "barcode ends with 7"
            elif last_digit == 4:
                task_to_execute = 3
                reason = "barcode ends with 4"
            else:
                rospy.loginfo("Barcode %d invalid ending - no task", barcode_number)
                
        # Log other faces but don't execute tasks
        if self.pending_face_detection == "Trump":
            rospy.loginfo("TRUMP DETECTED - no task")
        elif self.pending_face_detection == "Unknown":
            rospy.loginfo("UNKNOWN PERSON - no task")
        
        # Execute the selected task (only Obama or valid barcodes)
        if task_to_execute:
            rospy.loginfo("🚀 EXECUTING TASK %d - %s", task_to_execute, reason)
            self.task_pub.publish(Int32(data=task_to_execute))
            self.state = RobotState.TASK_SENT
            self.task_executed = True  # Mark task as executed
            
            # Start 8-second timer to return to parking mode
            if self.task_sent_timer is not None:
                self.task_sent_timer.shutdown()
            self.task_sent_timer = rospy.Timer(rospy.Duration(8.0), self.return_to_parking, oneshot=True)
            
            # Clear pending detections
            self.pending_face_detection = None
            self.pending_barcode_number = None
        else:
            # Clear detections so we can detect again
            self.pending_face_detection = None
            self.pending_barcode_number = None

    def qr_callback(self, msg):
        """Handle QR code detection messages"""
        current_time = rospy.Time.now()
        
        # Track perfect alignment timing
        if msg.perfect == 1:
            self.last_perfect_time = current_time
            
        # State machine handling
        if self.state == RobotState.PARKING:
            if msg.perfect == 1:
                self.state = RobotState.PARKED
                self.task_executed = False  # Reset task execution flag when newly parked
                rospy.loginfo("🅿️ PARKED - Ready for detection")
                # Cancel any existing timer since we now execute immediately
                if self.parked_timer is not None:
                    self.parked_timer.shutdown()
                    self.parked_timer = None
            else:
                # Still parking - send navigation commands
                self.handle_navigation_feedback(msg)
                
        elif self.state == RobotState.PARKED:
            # Check if we've been parked and stable for a while
            time_since_perfect = current_time - self.last_perfect_time
            if msg.perfect != 1:
                # Lost perfect alignment while parked - this might be a different QR code
                rospy.loginfo("Lost alignment while parked")
            
        elif self.state == RobotState.TASK_SENT:
            # Task executing - ignore all QR navigation
            pass

    def return_to_parking(self, event):
        """Called after 8 seconds to return to parking mode for continuous cycle"""
        rospy.loginfo("🔄 Task complete - returning to parking")
        self.state = RobotState.PARKING
        self.task_executed = False  # Reset task execution flag for next cycle
        # Clear any pending detections to start fresh
        self.pending_face_detection = None
        self.pending_barcode_number = None

    def make_decision(self, event):
        """Called after a delay to make task decision"""
        if self.state == RobotState.PARKED:
            rospy.loginfo("⏰ 5-second timer expired - making decision now!")
            rospy.loginfo("Current detections - Face: %s, Barcode: %s", 
                         self.pending_face_detection, self.pending_barcode_number)
            self.select_and_execute_task()

    def face_callback(self, msg):
        """Handle face detection messages - only Obama executes, others just log"""
        face_id = msg.data
        
        # Ignore all readings while executing task
        if self.state == RobotState.TASK_SENT:
            return
            
        # Ignore if task already executed this parking session
        if self.task_executed:
            return
        
        if self.state == RobotState.PARKED:
            # Only Obama triggers immediate execution
            if face_id == 1:
                self.pending_face_detection = "Obama"
                rospy.loginfo("👤 Obama detected - EXECUTING")
                self.select_and_execute_task()
            elif face_id == 2:
                self.pending_face_detection = "Trump"  
                rospy.loginfo("👤 Known face (not Obama) detected - LOGGING ONLY")
                self.select_and_execute_task()  # This will log but not execute
            else:
                self.pending_face_detection = "Unknown"
                rospy.loginfo("👤 Unknown face detected - LOGGING ONLY")
                self.select_and_execute_task()  # This will log but not execute
                
        elif self.state == RobotState.PARKING:
            # Store face detection for when we're parked
            if face_id == 1:
                self.pending_face_detection = "Obama"
                rospy.loginfo("👤 Obama detected while moving")
            elif face_id == 2:
                self.pending_face_detection = "Trump"
                rospy.loginfo("👤 Known face detected while moving")
            else:
                self.pending_face_detection = "Unknown"
                rospy.loginfo("👤 Unknown face detected while moving")

    def barcode_callback(self, msg):
        """Handle barcode number detection - immediate execution when parked"""
        try:
            barcode_number = msg.data
            
            # Ignore all readings while executing task
            if self.state == RobotState.TASK_SENT:
                return
                
            # Ignore if task already executed this parking session
            if self.task_executed:
                return
            
            if self.state == RobotState.PARKED:
                # Execute immediately when we detect barcode while parked
                self.pending_barcode_number = barcode_number
                last_digit = barcode_number % 10
                rospy.loginfo("🏷️ Barcode %d (ends with %d) - EXECUTING", barcode_number, last_digit)
                self.select_and_execute_task()
                    
            elif self.state == RobotState.PARKING:
                # Store barcode for when we're parked
                self.pending_barcode_number = barcode_number
                last_digit = barcode_number % 10
                rospy.loginfo("🏷️ Barcode %d (ends with %d) detected while moving", barcode_number, last_digit)
                
        except Exception as e:
            rospy.logerr("Error in barcode callback: %s", str(e))

    def run(self):
        """Main execution loop"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Task Manager shutting down")

def main():
    try:
        task_manager = TaskManager()
        task_manager.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()