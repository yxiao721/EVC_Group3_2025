#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int32, Float32
from encoderDriver import WheelEncoderDriver

GPIO_MOTOR_ENCODER_1 = 18  # BCM pin
GPIO_MOTOR_ENCODER_2 = 19

class EncoderNode:
    def __init__(self):
        rospy.init_node('encoder_node', anonymous=False)

        self.encoder_L = WheelEncoderDriver(GPIO_MOTOR_ENCODER_1)
        self.encoder_R = WheelEncoderDriver(GPIO_MOTOR_ENCODER_2)

        # Updated topic names
        self.pub_ticks = rospy.Publisher('/encoder/ticks', Int32, queue_size=10)
        self.pub_dt_L = rospy.Publisher('/encoder/dt_L', Float32, queue_size=10)
        self.pub_dt_R = rospy.Publisher('/encoder/dt_R', Float32, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            avg_ticks = (self.encoder_L._ticks + self.encoder_R._ticks) // 2

            dt_L = self.encoder_L.get_dt()
            dt_R = self.encoder_R.get_dt()

            # Default to 0.0 if dt is None
            dt_L_value = dt_L if dt_L is not None else 0.0
            dt_R_value = dt_R if dt_R is not None else 0.0

            # rospy.loginfo("Publishing -> ticks: %d | dt_L: %.4f | dt_R: %.4f", avg_ticks, dt_L_value, dt_R_value)

            self.pub_ticks.publish(avg_ticks)
            self.pub_dt_L.publish(dt_L_value)
            self.pub_dt_R.publish(dt_R_value)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = EncoderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
