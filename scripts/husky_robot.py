#!/usr/bin/env python
from operator import truediv
from turtle import distance
import rospy
from std_msgs.msg import Header, Bool
from genpy import Time
from geometry_msgs.msg import Twist, Point, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, pi, radians, sqrt
from husky_line_coverage.helpers import load_routes

DISTANCE_EPSILON = 0.4
ANGULAR_EPSILON = radians(5)

LINEAR_SPEED = 0.5
LINEAR_LOW_SPEED = 0.1

ANGULAR_SPEED = radians(12)
ANGULAR_LOW_SPEED = radians(2)

class HuskyRobot():
    def __init__(self) -> None:
        rospy.init_node('husky_robot_control', anonymous=True)

        # Params
        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "slc_beta2_atsp/slc_beta2_atsp_route")
        self.routes = load_routes(self.route_file, self.node_file)

        # Publishers
        self.vel_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.odom_subscriber = rospy.Subscriber('/odom2', Odometry, self.odom_callback)
        self.pause_subscriber = rospy.Subscriber("/is_pause", Bool, self.pause_callback)
        rospy.on_shutdown(self.shutdown_hook)

        # Vars
        self.rate = rospy.Rate(5)
        self.ctrl_c = False
        self.is_pause = True

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0
        # Odom
        self.odom = Pose()
        self.odom.orientation = Quaternion(0, 0, 0, 1)
        self.yaw = 0.0

    # Callbacks
    def shutdown_hook(self):
        self.ctrl_c = True

    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose
        q = self.odom.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def pause_callback(self, msg: Bool):
        self.is_pause = msg.data
        if self.is_pause:
            self.stop_robot()
    

    # Helpers
    def is_running(self) -> bool:
        return not self.ctrl_c

    def is_pause(self) -> bool:
        return self.is_pause

    def get_current_position(self) -> Point:
        return self.odom.position

    def calculate_distance(self, p1: Point, p2: Point):
        return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def calculate_angle(self, p1: Point, p2: Point):
        return atan2(p2.y - p1.y, p2.x - p1.x)

    def calculate_diff_angle(self, a1, a2):
        return self.normalize_angle(a2 - a1)

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    # Control robots
    def stop_robot(self) -> None:
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_publisher.publish(self.cmd)
        self.rate.sleep()

    # Rotate robot
    def rotate(self, angle):
        print("  - Rotate ", angle)
        rotated_angle = 0.0
        target_angle = self.yaw + angle
        last_yaw = self.yaw

        while rotated_angle < abs(angle) and self.is_running():
            if self.is_pause:
                self.rate.sleep()
                continue
            
            self.cmd.angular.z = min(ANGULAR_SPEED, max(abs(self.yaw - target_angle), 0.1))
            if angle < 0:
                self.cmd.angular.z = -self.cmd.angular.z
            self.vel_publisher.publish(self.cmd)

            self.rate.sleep()
            rotated_angle += abs(self.yaw - last_yaw)
            last_yaw = self.yaw
        
        self.stop_robot()

    def rotate_to_point(self, target_point: Point):
        while self.is_running():
            if self.is_pause:
                self.rate.sleep()
                continue
            # Calculate different angular
            current_point = self.get_current_position()
            target_angle = self.calculate_angle(current_point, target_point)
            diff_angle = self.calculate_diff_angle(self.yaw, target_angle)
            # Set break point
            if abs(diff_angle) < ANGULAR_EPSILON:
                break
            # Calculate angular cmd
            self.cmd.angular.z = min(ANGULAR_SPEED, max(abs(abs(diff_angle) - ANGULAR_SPEED), ANGULAR_LOW_SPEED))
            if diff_angle < 0:
                self.cmd.angular.z = -self.cmd.angular.z
            # Rotate robot
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

        self.stop_robot()


    def go_to_point(self, target_point: Point):
        current_point = self.get_current_position()
        print("-----------")
        print("Move to:", target_point.x, target_point.y)
        print("Current point: ", current_point.x, current_point.y)

        current_point = self.get_current_position()
        diff_distance = self.calculate_distance(current_point, target_point)
        if diff_distance < DISTANCE_EPSILON:
            return

        # Rotate robot to next point (OLD)
        # target_angle = self.calculate_angle(current_point, target_point)
        # diff_angle = self.calculate_diff_angle(self.yaw, target_angle)
        # # print("  ", degrees(self.gps_yaw), degrees(target_angle), degrees(diff_angle))
        # self.rotate(diff_angle)

        self.rotate_to_point(target_point)

        # Calculate distance to next point
        current_point = self.get_current_position()
        diff_distance = self.calculate_distance(current_point, target_point)
        last_diff_distance = diff_distance + 0.1
        max_distance = 1.01 * diff_distance
        last_position = current_point
        print("  - Distance to next point: ", diff_distance)

        d = 0.0
        while (d < max_distance 
            and diff_distance > DISTANCE_EPSILON
            and self.is_running()):
            # Check if pause
            if self.is_pause:
                self.rate.sleep()
                continue

            # Calculate linear vel
            self.cmd.linear.x = min(LINEAR_SPEED, max(diff_distance/2.0, LINEAR_LOW_SPEED))
            
            # Calculate angular vel
            target_angle = self.calculate_angle(current_point, target_point)
            diff_angle = self.calculate_diff_angle(self.yaw, target_angle)
            # print("  ", degrees(self.yaw), degrees(target_angle), degrees(diff_angle))
            if abs(diff_angle) > radians(3):
                self.cmd.angular.z = ANGULAR_LOW_SPEED if diff_angle > 0 else -ANGULAR_LOW_SPEED
            else:
                self.cmd.angular.z = 0.0
            
            # Move robot
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()
            # Recalcualte distance
            current_point = self.get_current_position()
            d += self.calculate_distance(last_position, current_point)
            last_diff_distance = diff_distance
            diff_distance = self.calculate_distance(current_point, target_point)
            last_position = current_point
        
        self.stop_robot()
        self.print_diff_points(current_point, target_point)

        
    def print_diff_points(self, current_point: Point, target_point: Point):
        print("  Current: (", current_point.x, ",", current_point.y, ")")
        print("  Target: (", target_point.x, ",", target_point.y, ")")
        print("  Diff(x,y): ", target_point.x - current_point.x, target_point.y - current_point.y)
        print("  Distance: ", self.calculate_distance(current_point, target_point))
        
    def start(self):
        rospy.wait_for_message("/odom2", Odometry)
        rospy.wait_for_message("/is_pause", Bool)

        zero_x = self.routes[0]["x"]
        zero_y = self.routes[0]["y"]

        for i in range(len(self.routes)):
            node = self.routes[i]
            point = Point(node["x"] - zero_x, node["y"] - zero_y, 0)
            self.go_to_point(point)

if __name__ == "__main__":
    robot = HuskyRobot()
    robot.start()
