#!/usr/bin/env python

import roslib
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Vector3
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

from geodesy import utm
from math import atan2, degrees, pi, radians, sqrt

class Test_Odom():
    def __init__(self) -> None:
        self.tf_listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()


        self.imu_subscriber = rospy.Subscriber('/imu_um7/mag', MagneticField, self.compass_callback)
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        self.odom_publisher = rospy.Publisher("/odom2", Odometry, queue_size=1)

        # UNCC
        self.init_point = utm.fromLatLong(35.3072185, -80.735079).toPoint()
        # WW
        self.init_point = utm.fromLatLong(35.30778316666667, -80.735224).toPoint()

        self.rate = rospy.Rate(10)

        # GPS
        self.lat = 0.0
        self.long = 0.0
        self.alt = 0.0

        # Compass
        self.compass_x = 0.0
        self.compass_y = 0.0
        self.compass_z = 0.0


    def compass_callback(self, msg: MagneticField):
        self.compass_x = msg.magnetic_field.x
        self.compass_y = msg.magnetic_field.y
        self.compass_z = msg.magnetic_field.z

    def gps_callback(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude

    def get_current_position(self) -> Point:
        point = utm.fromLatLong(self.lat, self.long).toPoint()
        return Point(point.x - self.init_point.x, point.y - self.init_point.y, 0.0)

    def start(self):
        while not rospy.is_shutdown():
            current_point = self.get_current_position()
            # print(quaternion_from_euler(self.compass_x, self.compass_y, self.compass_z))

            self.br.sendTransform(
                (0, 0, 0),
                # quaternion_from_euler(0, 0, radians(-91)),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "map2",
                "odom"
            )

            self.br.sendTransform(
                (current_point.x, current_point.y, 0.0),
                quaternion_from_euler(0, 0, self.compass_z),
                # (0, 0, 0, 1),
                rospy.Time.now(),
                "robot2",
                "map2"
            )
            self.rate.sleep()


            q = quaternion_from_euler(self.compass_x, self.compass_y, self.compass_z)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = current_point.x
            odom.pose.pose.position.y = current_point.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            self.odom_publisher.publish(odom)

if __name__ == "__main__":
    rospy.init_node("test_odom", anonymous=True)
    t = Test_Odom()
    t.start()
