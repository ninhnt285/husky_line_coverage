#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
from geodesy import utm
from math import atan2, pi

class HuskyRobot():
    def __init__(self) -> None:
        rospy.init_node('husky_robot_control', anonymous=True)
        # Publishers
        self.vel_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        # Vars
        self.rate = rospy.Rate(10)

        self.yaw = 0.0
        self.lat = 0.0
        self.long = 0.0
        self.alt = 0.0

    def imu_callback(self, msg: Imu):
        quaternion = msg.orientation
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def gps_callback(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude

    def relocate(self):
        self.rate.sleep()
        start_point = utm.fromLatLong(self.lat, self.long)
        self.go_straight(0.5)
        end_point = utm.fromLatLong(self.lat, self.long)
        # theta = atan2(end_point., dx)

    def rotate(self, angle):
        pass

    def go_straight(self, distance):
        pass

    def go_to_point(self, target_point, type="gps"):
        pass