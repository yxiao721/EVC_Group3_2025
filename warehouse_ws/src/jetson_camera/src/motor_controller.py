#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32, Float32
from motorDriver import DaguWheelsDriver
import time

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=False)

        # Start with average gain 0.65, both equal
        self.avg_gain_target = 0.65
        self.gain_L = 0.62
        self.gain_R = 0.6

        # Parameters
        self.ticks_per_cm = rospy.get_param('motor/ticks_per_cm', 6.70)
        self.ticks_per_deg = rospy.get_param('motor/ticks_per_deg', 0.77)
        self.velocity = rospy.get_param('motor/velocity', 0.5)

        self.motor = DaguWheelsDriver()
        self.current_ticks = 0
        self.start_ticks = 0

        self.dt_L = 0.0
        self.dt_R = 0.0
        self.dt_L_old = -1.0
        self.dt_R_old = -1.0
        self.speed = 1

        # Subscriptions with queue_size=1 and tcp_nodelay for fresh data
        rospy.Subscriber('/encoder/ticks', Int32, self.encoder_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/move', Int32, self.move_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rotate', Int32, self.rotate_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/encoder/dt_L', Float32, self.dt_L_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/encoder/dt_R', Float32, self.dt_R_callback, queue_size=1, tcp_nodelay=True)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("MotorController initialized with gain_L: %.3f, gain_R: %.3f",
                      self.gain_L, self.gain_R)

    def encoder_callback(self, msg):
        self.current_ticks = msg.data

    def dt_L_callback(self, msg):
        self.dt_L = msg.data

    def dt_R_callback(self, msg):
        self.dt_R = msg.data

    def _adjust_gains(self):
        # If either dt is zero, motor stuck, set both gains to 1.0
        # if self.dt_L == 0.0 or self.dt_R == 0.0:
        #     self.gain_L = 1.0
        #     self.gain_R = 1.0
        #     rospy.logwarn("Motor stuck detected (dt=0). Gains set to 1.0.")
            # return

        # Otherwise, adjust gains incrementally, keeping average at avg_gain_target
        dt_total = self.dt_L + self.dt_R
        
        if self.dt_L > 0 and self.dt_R > 0 and self.speed > 0:
            error = self.dt_L - self.dt_R
            correction_factor = 5  # as requested

            # Incremental update for gain_L
            delta = correction_factor * error
            new_gain_L = self.gain_L + delta
            # Compute gain_R to preserve average
            new_gain_R = 2.0 * self.avg_gain_target - new_gain_L

            # Clamp to [0.0, 1.0]
            new_gain_L = max(0.4, min(1.0, new_gain_L))
            new_gain_R = max(0.4, min(1.0, new_gain_R))

            self.gain_L = new_gain_L
            self.gain_R = new_gain_R

            rospy.loginfo("Adjusted gains -> gain_L: %.3f, gain_R: %.3f", self.gain_L, self.gain_R)
        self.dt_L_old = self.dt_L;
        self.dt_R_old = self.dt_R;


    def move_callback(self, msg):
        distance_cm = msg.data
        direction = 1 if distance_cm >= 0 else -1
        target_ticks = abs(distance_cm) * self.ticks_per_cm

        rospy.loginfo("Moving %s for %d cm (%d ticks)",
                      "forward" if direction > 0 else "backward",
                      abs(distance_cm), target_ticks)

        self.start_ticks = self.current_ticks
        start_time = time.time()

        # Initial wheel speeds
        self.motor.set_wheels_speed(self.gain_L * self.velocity * direction,
                                    self.gain_R * self.velocity * direction)

        while not rospy.is_shutdown() and abs(self.current_ticks - self.start_ticks) < target_ticks:
            elapsed_time = time.time() - start_time
            if elapsed_time > 0.0:
                ticks_travelled = abs(self.current_ticks - self.start_ticks)
                distance_travelled = ticks_travelled / self.ticks_per_cm
                speed_cm_per_s = distance_travelled / elapsed_time
                self.speed = speed_cm_per_s
                rospy.loginfo("Speed: %.2f cm/s", speed_cm_per_s)

            self._adjust_gains()
            # Update wheel speeds after adjusting gains
            self.motor.set_wheels_speed(self.gain_L * self.velocity * direction,
                                        self.gain_R * self.velocity * direction)

            rospy.sleep(0.1)  # adjust more frequently

        self.motor.set_wheels_speed(0, 0)
        rospy.loginfo("Move complete")

    def rotate_callback(self, msg):
        angle_deg = msg.data
        direction = 1 if angle_deg >= 0 else -1
        target_ticks = abs(angle_deg) * self.ticks_per_deg

        rospy.loginfo("Rotating %s for %d degrees (%d ticks)",
                      "clockwise" if direction > 0 else "counterclockwise",
                      abs(angle_deg), target_ticks)

        self.start_ticks = self.current_ticks

        self.motor.set_wheels_speed(-self.gain_L * self.velocity * direction,
                                    self.gain_R * self.velocity * direction)

        while not rospy.is_shutdown() and abs(self.current_ticks - self.start_ticks) < target_ticks:
            self._adjust_gains()
            self.motor.set_wheels_speed(-self.gain_L * self.velocity * direction,
                                        self.gain_R * self.velocity * direction)

            rospy.sleep(0.1)  # adjust more frequently during rotation

        self.motor.set_wheels_speed(0, 0)
        rospy.loginfo("Rotation complete")

    def shutdown_hook(self):
        rospy.loginfo("Shutting down motors.")
        self.motor.set_wheels_speed(0, 0)
        self.motor.close()

if __name__ == '__main__':
    try:
        MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass