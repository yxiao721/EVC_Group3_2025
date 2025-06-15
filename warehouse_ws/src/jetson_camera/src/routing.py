#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32
import time

class TaskExecutorNode:
    def __init__(self):
        rospy.init_node('task_executor_node', anonymous=True)

        # PARAMETERS - Adjust these as needed
        self.move_distance = 17      # cm for fixed movement distance
        self.command_delay = 3.0      # seconds between commands
        
        # Publishers for movement commands
        self.move_pub = rospy.Publisher('/move', Int32, queue_size=1)
        self.rotate_pub = rospy.Publisher('/rotate', Int32, queue_size=1)

        # Subscriber for task commands
        self.task_sub = rospy.Subscriber('/task', Int32, self.task_callback, queue_size=1)
        
        rospy.loginfo("Task Executor Node started")
        rospy.loginfo("Parameters: move_distance=%dcm, command_delay=%.1fs", 
                     self.move_distance, self.command_delay)
        rospy.loginfo("Listening for tasks on /task channel (1, 2, or 3)")
    
    def execute_command(self, command_type, value):
        """Execute a movement command with delay"""
        if command_type == "move":
            rospy.loginfo("Moving %d cm", value)
            self.move_pub.publish(Int32(data=value))
        elif command_type == "rotate":
            if value == 0:
                rospy.loginfo("No turn (continuing straight)")
            else:
                direction = "right" if value < 0 else "left"
                rospy.loginfo("Rotating %s %d degrees", direction, abs(value))
                self.rotate_pub.publish(Int32(data=value))
        
        # Wait for command to complete
        time.sleep(self.command_delay)
    
    def task_1(self):
        """Task 1: Turn around, move straight, turn LEFT, move straight"""
        rospy.loginfo("Executing Task 1: Turn around, straight, LEFT turn, straight")
        self.execute_command("rotate", 180)
        self.execute_command("move", self.move_distance)  # Move straight
        self.execute_command("rotate", 90)   # Turn LEFT 90°
        self.execute_command("move", self.move_distance)  # Move straight again
        rospy.loginfo("Task 1 completed")

    def task_2(self):
        """Task 2: Turn around, move straight, NO turn, move straight"""
        rospy.loginfo("Executing Task 2: Turn around, straight, NO turn, straight")
        
        self.execute_command("rotate", 180)  # Turn around 180°
        self.execute_command("move", self.move_distance)  # Move straight
        self.execute_command("move", self.move_distance)  # Move straight again
        
        rospy.loginfo("Task 2 completed")
    
    def task_3(self):
        """Task 3: Turn around, move straight, turn RIGHT, move straight"""
        rospy.loginfo("Executing Task 3: Turn around, straight, RIGHT turn, straight")
        
        self.execute_command("rotate", 180)  # Turn around 180°
        self.execute_command("move", self.move_distance)  # Move straight
        self.execute_command("rotate", -90)  # Turn RIGHT 90°
        self.execute_command("move", self.move_distance)  # Move straight again
        
        rospy.loginfo("Task 3 completed")
    
    def task_callback(self, msg):
        """Handle incoming task messages"""
        task_number = msg.data
        
        rospy.loginfo("Received task command: %d", task_number)
        
        if task_number == 1:
            self.task_1()
        elif task_number == 2:
            self.task_2()
        elif task_number == 3:
            self.task_3()
        else:
            rospy.logwarn("Unknown task number: %d. Valid tasks are 1, 2, or 3", task_number)

if __name__ == "__main__":
    try:
        node = TaskExecutorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task Executor Node shut down")