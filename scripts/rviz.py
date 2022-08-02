#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Bool
from husky_line_coverage.helpers import load_routes
from math import pi, sqrt

import cv2

class Rviz_Support():
    def __init__(self) -> None:
        rospy.init_node('rviz_support')
        self.rate = rospy.Rate(1)

        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "./maps/route")
        self.map_image = rospy.get_param("map_image", "./map/woodward.png")
        self.routes = load_routes(self.route_file, self.node_file)
        self.path_shape = rospy.get_param("path_shape", "circle")

        self.arrived_subscriber = rospy.Subscriber("/arrived_index", Int16, self.arrived_callback)

        self.root_frame = "map"
        self.arrived_index = 0
        self.is_updated = False

        # Simulation
        self.is_simulation = rospy.get_param("is_simulation", False)
        self.is_bag = rospy.get_param("is_bag", False)
        if self.is_simulation:
            self.root_frame = "odom"

        if self.is_bag:
            self.root_frame = "map"

    def arrived_callback(self, msg: Int16):
        self.arrived_index = msg.data
        self.is_updated = False

    def draw_route(self):
        marker_pub = rospy.Publisher("route_marker", Marker, queue_size=10)

        image = Marker()
        image.header.frame_id = self.root_frame
        image.id = 0
        image.action = Marker.MODIFY
        image.pose.orientation.w = 1.0
        
        if self.path_shape == "circle":
            image.type = Marker.SPHERE_LIST
            image.scale.x = 0.5
            image.scale.y = 0.5
        else:
            image.type = Marker.LINE_STRIP
            image.scale.x = 0.2
            image.scale.y = 0.2
        
        image.scale.z = 0.01
        image.ns = "map"

        new_color = ColorRGBA(1.0, 0.0, 0.0, 1.0) # Red
        old_color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # Green

        center_point = Point(self.routes[0]["x"], self.routes[0]["y"], 0)

        for i in range(len(self.routes) - 1):
            start_node = self.routes[i]
            next_node = self.routes[i+1]

            if self.path_shape == "circle":
                d = sqrt((start_node["x"] - next_node["x"])**2 + (start_node["y"] - next_node["y"])**2)
                c = int(d / 1.5) + 1
                dx = (next_node["x"] - start_node["x"]) / float(c)
                dy = (next_node["y"] - start_node["y"]) / float(c)
                for j in range(c + 1):
                    color = new_color
                    if i < self.arrived_index:
                        color = old_color
                    image.colors.append(color)

                    p = Point()
                    p.x = start_node["x"] - center_point.x + dx * float(j)
                    p.y = start_node["y"] - center_point.y + dy * float(j)
                    if i < self.arrived_index:
                        p.z = -0.1
                    else:
                        p.z = -0.2
                    image.points.append(p)
            else:
                start_point = Point(start_node["x"] - center_point.x, start_node["y"] - center_point.y, -0.2)
                end_point = Point(next_node["x"] - center_point.x, next_node["y"] - center_point.y, -0.2)

                if i < self.arrived_index:
                    image.colors.append(old_color)
                    image.colors.append(old_color)
                    start_point.z = -0.1
                    end_point.z = -0.1
                else:
                    image.colors.append(new_color)
                    image.colors.append(new_color)

                image.points.append(start_point)
                image.points.append(end_point)

        marker_pub.publish(image)

    def draw_map(self):
        marker_pub = rospy.Publisher("map_marker", Marker, queue_size=10)
        map = cv2.imread(self.map_image, cv2.COLOR_BGR2RGB)
        map_size = map.shape
        scale = 0.2
        step = 5
        center_point = Point(560.0, 530.0, 0.0)
        rotate_angle = 0

        image = Marker()
        image.header.frame_id = self.root_frame
        image.id = 0
        image.type = Marker.CUBE_LIST
        image.action = Marker.MODIFY
        
        image.pose.orientation.z = pi / 180.0 * rotate_angle
        image.pose.orientation.w = 1.0

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
                color.a = 0.2
                image.colors.append(color)

                p = Point()
                p.x = x * scale
                p.y = y * scale
                p.z = -1
                image.points.append(p)

        marker_pub.publish(image)

    def start(self):
        while not rospy.is_shutdown():
            if not self.is_updated:
                self.draw_route()
                self.draw_map()
                self.rate.sleep()


if __name__ == "__main__":
    prog = Rviz_Support()
    prog.start()