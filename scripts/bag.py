#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class Bag_Support():
    def __init__(self) -> None:
        rospy.init_node("bag_support", anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        rospy.Subscriber("odom2", Odometry, self.get_odom_callback)
        rospy.Subscriber("laser_cloud_less_flat", PointCloud2, self.get_point_callback)

        self.pc_publisher = rospy.Publisher("laser_cloud_less_flat2", PointCloud2, queue_size=100)
        self.rate = rospy.Rate(30)

        self.is_started = False
        self.start_secs = 0.0
        self.start_nsecs = 0.0


    def get_point_callback(self, msg: PointCloud2):
        if not self.is_started:
            self.start_secs = msg.header.stamp.secs
            self.start_nsecs = msg.header.stamp.nsecs
            self.is_started = True
        
        # msg.header.stamp.secs = msg.header.stamp.secs - self.start_secs - 1

        msg.header.stamp = rospy.Time.now()

        self.pc_publisher.publish(msg)

    def get_odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.tf_br.sendTransform(
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                rospy.Time.now(),
                "base_loam",
                "map"
            )

    def start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            continue

if __name__ == "__main__":
    bag_supporter = Bag_Support()
    bag_supporter.start()