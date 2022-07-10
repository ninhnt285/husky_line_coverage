#!/usr/bin/env python

import rospy
from husky_robot import HuskyRobot
from geodesy import utm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import pi, sqrt
from std_msgs.msg import ColorRGBA

class RouteController():
    def __init__(self, robot: HuskyRobot = None):
        # rospy.init_node('route_controller', anonymous=True)
        self.robot = robot

        self.data_dir = rospy.get_param("data_dir", "/home/ninhnt/coverage_ws/LineCoverage-dataset/icra20_dataset/ww/")

        # self.json_file = self.data_dir + rospy.get_param("route_json", "slc_beta2_atsp/route.json")
        # self.xml_file = self.data_dir + rospy.get_param("route_xml", "slc_beta2_atsp/slc_beta2_atsp_route.kml")
        self.nodes_file = self.data_dir + rospy.get_param("nodes_file", "node_data")
        self.route_file = self.data_dir + rospy.get_param("route_file", "slc_beta2_atsp/slc_beta2_atsp_route")
        
        self.nodes = {}
        self.routes = []

        self.load_file()

    def load_file(self):
        with open(self.nodes_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                params = line.split()
                self.nodes[params[0]] = {"lat": params[3], "long": params[4]}

        with open(self.route_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                params = line.split()
                self.routes.append(params[0])

    def print_route(self):
        print(self.routes)

    def print_nodes(self):
        zero_point = utm.fromLatLong(35.307076, -80.7343572).toPoint()
        for i in range(len(self.routes)):
            node = self.nodes[self.routes[i]]
            point = self.get_point_from_node(node)
            print(point.x - zero_point.x, point.y - zero_point.y)


    def get_point_from_node(self, node):
        return utm.fromLatLong(float(node["lat"]), float(node["long"])).toPoint()

    def start(self):
        self.print_nodes()

        self.robot.relocate()

        # # for i in range(len(self.routes)):
        # #     node = self.nodes[self.routes[i]]
        # #     point = self.get_point_from_node(node)

        # #     print("Move to point " + str(i) + ": ", point.x, point.y )
        # #     self.robot.go_to_point(point)

        # Go back to first node
        print("Move back to start point")
        node = self.nodes[self.routes[0]]
        point = self.get_point_from_node(node)
        self.robot.go_to_point(point)


if __name__ == "__main__":
    rc = RouteController()