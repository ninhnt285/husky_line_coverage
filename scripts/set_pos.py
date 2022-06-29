#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from math import pi

def set_position_callback(msg):
    print(msg.pose.pose)
    

if __name__ == "__main__":
    rospy.init_node("tf2_set_location")

    rospy.Subscriber("set_position", PoseWithCovarianceStamped, set_position_callback)
    rospy.spin()

    # broadcaster = tf2_ros.StaticTransformBroadcaster()
    # static_transformStamped = TransformStamped()
    
    # static_transformStamped.header.stamp = rospy.Time.now()
    # static_transformStamped.header.frame_id = "map"
    # static_transformStamped.child_frame_id = "base_loam"

    # static_transformStamped.transform.translation.x = 1.0
    # static_transformStamped.transform.translation.y = 1.0
    # static_transformStamped.transform.translation.z = 0.0

    # static_transformStamped.transform.rotation.x = 0.0
    # static_transformStamped.transform.rotation.y = 0.0
    # static_transformStamped.transform.rotation.z = 0.0
    # static_transformStamped.transform.rotation.w = 1.0

    # while not rospy.is_shutdown():
    #     broadcaster.sendTransform(static_transformStamped)
    #     print("Post")

