#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from geodesy import utm
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from husky_line_coverage.helpers import load_routes
from math import radians, sin, cos

class Odom():
    def __init__(self) -> None:
        rospy.init_node("odom2_controller", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        # Load Params
        # self.position_input = rospy.get_param("position_input", "gps")
        # self.orientation_input = rospy.get_param("orientation_input", "imu")
        self.is_simulation = rospy.get_param("is_simulation", "false")
        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "./maps/route")

        # Subscribe
        rospy.Subscriber("set_position", PoseWithCovarianceStamped, self.set_position_callback)

        # Publisher
        self.odom_publisher = rospy.Publisher("/odom2", Odometry, queue_size=1)

        # Variables
        self.rate = rospy.Rate(1)

        # New Coords
        self.zero_position = Point(0, 0, 0)
        self.zero_orientation = Quaternion(0, 0, 0, 1)
        self.zero_yaw = 0.0

        # Robot
        self.robot_position = Point(0, 0, 0)
        self.robot_orienation = Quaternion(0, 0, 0, 1)
        

    def rotate_point(self, angle: float, p: Point, pivot: Point = Point(0, 0, 0)) -> Point:
        s = sin(angle)
        c = cos(angle)

        p.x -= pivot.x
        p.y -= pivot.y

        xnew = p.x * c - p.y * s
        ynew = p.x * s + p.y * c

        p.x = xnew + pivot.x
        p.y = ynew + pivot.y
        return p

    ## Callbacks
    def set_position_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        old_position = self.robot_position
        new_position = msg.pose.pose.position

        # print("Robot Position: ", old_position.x, old_position.y)
        # print("New Position:", new_position.x, new_position.y)

        # Re-calculate odom2_init
        robot_q = self.robot_orienation
        (_, _, robot_yaw) = euler_from_quaternion([robot_q.x, robot_q.y, robot_q.z, robot_q.w])
        angle = yaw - robot_yaw
        # print("Yaw: ", yaw, robot_yaw)
        # print("Rotate Angle: ", angle)
        rotated_point = self.rotate_point(angle, Point(0, 0, 0), old_position)
        # print("Rotated Point:", rotated_point.x, rotated_point.y)

        zero_q = quaternion_from_euler(0, 0, angle)
        self.zero_yaw = angle
        self.zero_position = Point(rotated_point.x + new_position.x - old_position.x, rotated_point.y + new_position.y - old_position.y, 0)
        # print("New zero point:", self.zero_position.x, self.zero_position.y)
        self.zero_orientation = Quaternion(*zero_q)


    ## Functions
    def calculate_new_position(self, p: Point):
        new_point = self.rotate_point(self.zero_yaw, p, self.zero_position)
        return new_point


    def start(self):
        while not rospy.is_shutdown():
            robot_frame = "base_loam"
            root_frame = "map"
            if self.is_simulation:
                robot_frame = "base_link"
                root_frame = "odom"

            # odom2_init tf
            self.tf_br.sendTransform(
                (self.zero_position.x, self.zero_position.y, self.zero_position.z),
                (self.zero_orientation.x, self.zero_orientation.y, self.zero_orientation.z, self.zero_orientation.w),
                rospy.Time.now(),
                "map2",
                root_frame
            )

            try:
                (trans, rot) = self.tf_listener.lookupTransform(root_frame, robot_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                continue
            self.robot_position = Point(*trans)
            self.robot_orienation = Quaternion(*rot)

            # robot2 tf
            self.tf_br.sendTransform(
                (trans),
                (rot),
                rospy.Time.now(),
                "robot2",
                "map2"
            )

            # Publish odom2 topic
            try:
                (trans2, rot2) = self.tf_listener.lookupTransform(root_frame, "robot2", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                continue

            odom2 = Odometry()
            odom2.header.frame_id = root_frame
            odom2.pose.pose.position = Point(*trans2)
            odom2.pose.pose.orientation = Quaternion(*rot2)
            self.odom_publisher.publish(odom2)

            self.rate.sleep()


if __name__ == "__main__":
    odom = Odom()
    odom.start()
