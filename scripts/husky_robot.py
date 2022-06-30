#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

class HuskyRobot():
    def __init__(self) -> None:
        rospy.init_node('husky_robot_control', anonymous=True)
        # Publishers
        self.vel_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.gps_subscriber = rospy.Subscriber('/gps/fix')