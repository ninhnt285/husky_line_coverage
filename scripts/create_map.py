#! /usr/bin/env python

import rospy
import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import cos, pi, sin

if __name__ == "__main__":
    rospy.init_node('rviz_marker')
    marker_pub = rospy.Publisher("/map_marker", Marker, queue_size=10)

    # Load map
    map = cv2.imread("./maps/woodward.png", cv2.COLOR_BGR2RGB)

    map_size = map.shape
    scale = 0.2
    step = 5
    print(map_size)


    center_point = Point(205.0, 512.0, 0.0)
    rotate_angle = -49.0

    # center_point = Point(float(map_size[1]) / 2.0, float(map_size[0]) / 2.0, 0.0)
    # rotate_angle = 0.0


    image = Marker()
    image.header.frame_id = "map"
    image.id = 0
    image.type = Marker.CUBE_LIST
    image.action = Marker.MODIFY

    image.pose.orientation.z = pi / 180.0 * rotate_angle
    image.pose.orientation.w = 1.0

    image.pose.position.x = 0.0
    image.pose.position.y = 0.0
    image.pose.position.z = 0.0

    image.scale.x = 1
    image.scale.y = 1
    image.scale.z = 0.01

    image.ns = "map"

    for r in range(0, map_size[0], step):
        for c in range(0, map_size[1], step):
            x = float(c) - center_point.x
            y = center_point.y - float(r)

            color = ColorRGBA()
            color.r = float(map[r][c][0]) / 255.0
            color.g = float(map[r][c][1]) / 255.0
            color.b = float(map[r][c][2]) / 255.0
            color.a = 0.5
            image.colors.append(color)

            p = Point()
            p.x = x * scale
            p.y = y * scale
            p.z = 0
            image.points.append(p)


    # while not rospy.is_shutdown():
    marker_pub.publish(image)
    # rospy.sleep(1.0)