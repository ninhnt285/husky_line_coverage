#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import NavSatFix, MagneticField
from geodesy import utm
from husky_line_coverage.helpers import load_routes
from math import atan2, pi, sqrt
from tf.transformations import quaternion_from_euler

class Gps_Odom():
    def __init__(self) -> None:
        rospy.init_node("gps_odom", anonymous=True)

        # Load Params
        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "./maps/route")
        self.routes = load_routes(self.route_file, self.node_file)
        self.center_point = Point(self.routes[0]["x"], self.routes[0]["y"], 0)

        # Publisher
        self.odom_publisher = rospy.Publisher("/gps_odom", Odometry, queue_size=1)

        # Subscriber
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu/mag", MagneticField, self.mag_callback)
        rospy.Subscriber("/odom2", Odometry, self.odom_callback)

        self.current_point = Point(0,0,0)
        self.current_angle = 0.0
        self.total_error = 0.0
        self.error_count = 0.0
        self.rate = rospy.Rate(rospy.get_param("time_rate", 30))

    def cal_distance(self, p1: Point, p2: Point):
        return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def odom_callback(self, msg: Odometry):
        self.total_error = self.total_error + self.cal_distance(msg.pose.pose.position, self.current_point)
        self.error_count = self.error_count + 1.0

    def gps_callback(self, msg: NavSatFix):
        utm_point = utm.fromLatLong(msg.latitude, msg.longitude).toPoint()
        self.current_point = Point(utm_point.x - self.center_point.x, utm_point.y - self.center_point.y, 0)

    def mag_callback(self, msg: MagneticField):
        self.current_angle = atan2(-msg.magnetic_field.y, msg.magnetic_field.x) - 150.0/180.0 * pi

    def start(self):
        while not rospy.is_shutdown():
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.pose.pose.position = self.current_point

            q = quaternion_from_euler(0, 0, self.current_angle)
            odom.pose.pose.orientation = Quaternion(*q)
            self.odom_publisher.publish(odom)

            self.rate.sleep()

        print("AVG GPS Error: ", self.total_error / self.error_count)

if __name__ == "__main__":
    odom = Gps_Odom()
    odom.start()