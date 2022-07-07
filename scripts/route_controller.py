#!/usr/bin/env python

import rospy
import json
from husky_robot import HuskyRobot
from geodesy import utm

class RouteController():
    def __init__(self, robot: HuskyRobot = None):
        # rospy.init_node('route_controller', anonymous=True)
        self.robot = robot

        self.data_dir = rospy.get_param("data_dir", "/home/tnguy248/coverage_ws/LineCoverage-dataset/icra20_dataset/ww/")

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

    def get_point_from_node(self, node):
        return utm.fromLatLong(float(node["lat"]), float(node["long"])).toPoint()

    def start(self):
        self.robot.relocate()

        # for i in range(len(self.routes)):
        #     node = self.nodes[self.routes[i]]
        #     point = self.get_point_from_node(node)

        #     print("Move to point " + str(i) + ": ", point.x, point.y )
        #     self.robot.go_to_point(point)

        # Go back to first node
        node = self.nodes[self.routes[0]]
        point = self.get_point_from_node(node)
        self.robot.go_to_point(point)