#!/usr/bin/env python

import rospy
from route_controller import RouteController

if __name__ == "__main__":
    route_controller = RouteController()
    route_controller.start()
