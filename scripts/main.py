#!/usr/bin/env python

import rospy
from route_controller import RouteController
from husky_robot import HuskyRobot

if __name__ == "__main__":
    robot = HuskyRobot()
    route_controller = RouteController(robot)
    route_controller.start()
