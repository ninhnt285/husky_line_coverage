#!/usr/bin/env python

import rospy
from husky_line_coverage.husky_robot import HuskyRobot
from geodesy import utm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import pi, sqrt
from husky_line_coverage.helpers import load_routes


class RouteController():
    def __init__(self, robot: HuskyRobot = None):
        # rospy.init_node('route_controller', anonymous=True)
        self.robot = robot

        # self.json_file = self.data_dir + rospy.get_param("route_json", "slc_beta2_atsp/route.json")
        # self.xml_file = self.data_dir + rospy.get_param("route_xml", "slc_beta2_atsp/slc_beta2_atsp_route.kml")
        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "slc_beta2_atsp/slc_beta2_atsp_route")
        
        self.routes = load_routes(self.route_file, self.node_file)


    def get_point_from_node(self, node):
        return utm.fromLatLong(float(node["lat"]), float(node["long"])).toPoint()

    def start(self):
        zero_x = self.routes[0]["x"]
        zero_y = self.routes[0]["y"]

        for i in range(len(self.routes)):
            node = self.routes[i]
            point = Point(node["x"] - zero_x, node["y"] - zero_y, 0)
            self.robot.go_to_point(point)


if __name__ == "__main__":
    rc = RouteController()
    robot = HuskyRobot()
    rc.start()