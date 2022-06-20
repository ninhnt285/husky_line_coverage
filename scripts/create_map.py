#! /usr/bin/env python

import rospy
import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

if __name__ == "__main__":
    rospy.init_node('rviz_marker')
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

    # Load map
    map = cv2.imread("./maps/uncc.png", cv2.COLOR_BGR2RGB)
    map_size = map.shape
    ratio = 100
    scale = 1.0 / float(ratio)
    step = 5
    print(map_size)

    diff_pos = Point(0.0, 0.0, 0.0)

    image = Marker()
    image.header.frame_id = "my_frame"
    image.id = 0
    image.type = Marker.POINTS

    image.pose.orientation.w = 1.0
    image.pose.position.x = 0.0
    image.pose.position.y = 0.0
    image.pose.position.z = 0.0

    image.scale.x = scale * step
    image.scale.y = scale * step
    image.scale.z = 0.01

    image.ns = "map"

    for r in range(0, map_size[0], step):
        for c in range(0, map_size[1], step):
            color = ColorRGBA()
            color.r = float(map[r][c][0]) / 255.0
            color.g = float(map[r][c][1]) / 255.0
            color.b = float(map[r][c][2]) / 255.0
            color.a = 0.2
            image.colors.append(color)

            p = Point()
            p.x = diff_pos.x - float(c) * scale
            p.y = diff_pos.y - float(r) * scale
            p.z = -0.05
            image.points.append(p)


    while not rospy.is_shutdown():
        marker_pub.publish(image)
        rospy.sleep(1.0)