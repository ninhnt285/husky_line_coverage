#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Bool
from husky_line_coverage.helpers import load_routes
from math import pi, sqrt

class Rviz_Support():
    def __init__(self) -> None:
        rospy.init_node('rviz_support')

        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "./maps/route")
        self.routes = load_routes(self.route_file, self.node_file)
        
        self.root_frame = "map"

        # Simulation
        self.is_simulation = rospy.get_param("is_simulation", False)
        if self.is_simulation:
            self.root_frame = "odom"


    def draw_route(self):
        marker_pub = rospy.Publisher("route_marker", Marker, queue_size=10)

        image = Marker()
        image.header.frame_id = self.root_frame
        image.id = 0
        image.type = Marker.SPHERE_LIST
        image.action = Marker.MODIFY
        image.pose.orientation.w = 1.0
        image.scale.x = 0.5
        image.scale.y = 0.5
        image.scale.z = 0.01
        image.ns = "map"

        center_point = Point(self.routes[0]["x"], self.routes[0]["y"], 0)

        for i in range(len(self.routes) - 1):
            start_node = self.routes[i]
            next_node = self.routes[i+1]
            
            d = sqrt((start_node["x"] - next_node["x"])**2 + (start_node["y"] - next_node["y"])**2)
            c = int(d / 0.2) + 1
            dx = (next_node["x"] - start_node["x"]) / float(c)
            dy = (next_node["y"] - start_node["y"]) / float(c)
            for j in range(c):
                color = ColorRGBA()
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
                color.a = 0.5
                image.colors.append(color)

                p = Point()
                p.x = start_node["x"] - center_point.x + dx * float(j)
                p.y = start_node["y"] - center_point.y + dy * float(j)
                p.z = 0
                image.points.append(p)

        rospy.sleep(3)
        marker_pub.publish(image)


if __name__ == "__main__":
    prog = Rviz_Support()
    prog.draw_route()