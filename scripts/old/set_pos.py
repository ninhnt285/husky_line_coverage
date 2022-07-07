#!/usr/bin/env python

from tkinter import W
import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Quaternion, Point
from tf.transformations import *
from math import pi

def set_position_callback(msg):
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Set camera_init position
    rotate = euler_from_quaternion(
        [msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w]
    )
    origin = quaternion_from_euler(1.570759,0,1.570759+rotate[2])

    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "camera_init"
    static_transformStamped.transform.translation.x = msg.pose.pose.position.x
    static_transformStamped.transform.translation.y = msg.pose.pose.position.y
    static_transformStamped.transform.translation.z = msg.pose.pose.position.z
    static_transformStamped.transform.rotation.x = origin[0]
    static_transformStamped.transform.rotation.y = origin[1]
    static_transformStamped.transform.rotation.z = origin[2]
    static_transformStamped.transform.rotation.w = origin[3]

    # Set vel_loam position
    vel_loam_transform = TransformStamped()
    vel_loam_transform.header.stamp = rospy.Time.now()
    vel_loam_transform.header.frame_id = "camera_init"
    vel_loam_transform.child_frame_id = "vel_loam"
    vel_loam_transform.transform.translation = Point(0,0,0)
    vel_loam_transform.transform.rotation = Quaternion(0,0,0,1)

    # Set base_link position
    baselink_transform = TransformStamped()
    baselink_transform.header.stamp = rospy.Time.now()
    baselink_transform.header.frame_id = "map"
    baselink_transform.child_frame_id = "base_link"
    baselink_transform.transform.translation = msg.pose.pose.position
    baselink_transform.transform.rotation = msg.pose.pose.orientation

    if not rospy.is_shutdown():
        print("Set Position")
        broadcaster.sendTransform(static_transformStamped)
        broadcaster.sendTransform(vel_loam_transform)
        broadcaster.sendTransform(baselink_transform)

    

if __name__ == "__main__":
    rospy.init_node("tf2_set_location")

    rospy.Subscriber("set_position", PoseWithCovarianceStamped, set_position_callback)
    rospy.spin()

    



