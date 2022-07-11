#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class Joystick():
    def __init__(self) -> None:
        rospy.init_node("joy_action", anonymous=True)

        self.pause_publisher = rospy.Publisher("/is_pause", Bool, queue_size=5)
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.is_pause = True

        self.rate = rospy.Rate(10)

    def joy_callback(self, msg: Joy):
        # Triangle: Start/Resume
        if msg.buttons[2]:
            self.is_pause = False

        # X: Pause
        if msg.buttons[0]:
            self.is_pause = True

        self.pause_publisher.publish(self.pause_publisher)
        self.rate.sleep()

if __name__ == "__main__":
    joy = Joystick()
    while not rospy.is_shutdown():
        rospy.spin()