#!/usr/bin/env python

import rospy
import json
from husky_robot import HuskyRobot
from geodesy import utm

class RouteController():
    def __init__(self, robot: HuskyRobot = None):
        rospy.init_node('route_controller', anonymous=True)
        self.robot = robot

        self.json_file = rospy.get_param('route_file', "/home/ninhnt/coverage_ws/LineCoverage-dataset/icra20_dataset/ww/beta2_atsp/route.json")
        self.routes = []

        self.load_file()

    def load_file(self):
        with open(self.json_file) as f:
            self.routes = json.loads(f.read())

    def print_route(self):
        for line in self.routes:
            coordinates = line["geometry"]["coordinates"]
            print(coordinates)


    def start(self):
        coord = self.routes[0]["geometry"]["coordinates"][0]
        utm_point = utm.fromLatLong(coord[1], coord[0])
        print(utm_point.band, utm_point.zone)
        print(utm_point.toPoint())