#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class Joystick():
    def __init__(self) -> None:
        rospy.init_node("joy_action", anonymous=True)

        self.joy_topic = rospy.get_param("joy_topic", "/joy_teleop/joy")

        self.pause_publisher = rospy.Publisher("/is_pause", Bool, queue_size=5)
        self.joy_subscriber = rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)
        self.is_pause = True

        self.rate = rospy.Rate(rospy.get_param("time_rate", 30))

    def joy_callback(self, msg: Joy):
        old_value = self.is_pause

        # Triangle: Start/Resume
        if msg.buttons[2]:
            self.is_pause = False

        # X: Pause
        if msg.buttons[0]:
            self.is_pause = True

        if old_value != self.is_pause:
            if not self.is_pause:
                print("Start Moving")
            else:
                print("Pause Robot")
            print()
            self.pause_publisher.publish(self.is_pause)
            self.rate.sleep()

if __name__ == "__main__":
    joy = Joystick()
    while not rospy.is_shutdown():
        rospy.spin()