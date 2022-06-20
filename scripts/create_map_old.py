#! /usr/bin/env python

import rospy
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.init_node('rviz_marker')
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)

    # Load map
    map = cv2.imread("./woodward.png", cv2.COLOR_BGR2RGB)
    map_size = map.shape
    scale = 0.01
    print(map_size)

    diff_pos = Point()
    diff_pos.x = 0.0
    diff_pos.y = 0.0
    diff_pos.z = 0.0

    image = MarkerArray()

    for r in range(map_size[0]):
        for c in range(map_size[1]):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = r * map_size[1] + c
            marker.action = Marker.ADD
            marker.type = Marker.CUBE

            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            marker.color.r = float(map[r][c][0]) / 255.0
            marker.color.g = float(map[r][c][1]) / 255.0
            marker.color.b = float(map[r][c][2]) / 255.0
            marker.color.a = 1.0

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = diff_pos.x + float(c) * scale
            marker.pose.position.y = diff_pos.y + float(r) * scale
            marker.pose.position.z = 0.0
            
            image.markers.append(marker)


    while not rospy.is_shutdown():
        print("Publish")
        marker_pub.publish(image)
        rospy.rostime.wallsleep(1.0)
        break