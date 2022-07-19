#!/usr/bin/env python
from operator import truediv
from turtle import distance
import rospy
from std_msgs.msg import Bool, Int16
from genpy import Time
from geometry_msgs.msg import Twist, Point, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, pi, radians, sqrt
from husky_line_coverage.helpers import load_routes


DISTANCE_EPSILON = rospy.get_param("distance_epsilon")
LINEAR_MAX_SPEED = rospy.get_param("linear_max_speed")
LINEAR_MIN_SPEED = rospy.get_param("linear_min_speed")
LINEAR_SPEED_STEP = rospy.get_param("linear_speed_step")

ANGULAR_EPSILON = radians(rospy.get_param("angular_epsilon_degree"))
ANGULAR_MOVING_EPSILON = radians(rospy.get_param("angular_moving_epsilon_degree"))
ANGULAR_MAX_SPEED = radians(rospy.get_param("angular_max_speed_degree"))
ANGULAR_MIN_SPEED = radians(rospy.get_param("angular_min_speed_degree"))

class HuskyRobot():
    def __init__(self) -> None:
        rospy.init_node('husky_robot_control', anonymous=True)

        # Params
        self.node_file = rospy.get_param("node_file", "./maps/node_data")
        self.route_file = rospy.get_param("route_file", "slc_beta2_atsp/slc_beta2_atsp_route")
        self.routes = load_routes(self.route_file, self.node_file)

        # Publishers
        self.vel_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.arrived_publisher = rospy.Publisher('/arrived_index', Int16, queue_size=1)

        # Subscribers
        self.odom_subscriber = rospy.Subscriber('/odom2', Odometry, self.odom_callback)
        self.pause_subscriber = rospy.Subscriber("/is_pause", Bool, self.pause_callback)
        rospy.on_shutdown(self.shutdown_hook)

        # Vars
        self.rate = rospy.Rate(rospy.get_param("time_rate", 30))
        self.ctrl_c = False
        self.is_pause = True
        self.is_bag = not rospy.get_param("is_simulation", False) and not rospy.get_param("on_robot", False)

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

    def calculate_angular_vel(self, diff_angle, is_moving=False):
        speed = abs(diff_angle) * 0.5
        
        speed = min(ANGULAR_MAX_SPEED, speed)

        if not is_moving and speed < ANGULAR_MIN_SPEED:
            speed = ANGULAR_MIN_SPEED

        if diff_angle < 0:
            speed = -speed

        return speed

    def calculate_linear_vel(self, diff_distance, current_speed):
        speed = diff_distance * 0.5

        if speed > LINEAR_MAX_SPEED:
            speed = LINEAR_MAX_SPEED

        if current_speed + LINEAR_SPEED_STEP < speed:
            speed = current_speed + LINEAR_SPEED_STEP

        return speed

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
            self.cmd.angular.z = self.calculate_angular_vel(diff_angle)
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

        print("  Rotate to next point: ", diff_distance)
        self.rotate_to_point(target_point)

        # Calculate distance to next point
        current_point = self.get_current_position()
        diff_distance = self.calculate_distance(current_point, target_point)
        last_position = current_point
        print("  Distance to next point: ", diff_distance)

        d = 0.0
        while (diff_distance > DISTANCE_EPSILON
            and self.is_running()):
            # Check if pause
            if self.is_pause:
                self.rate.sleep()
                continue

            # Calculate linear vel
            self.cmd.linear.x = self.calculate_linear_vel(diff_distance, self.cmd.linear.x)
            
            # Calculate angular vel
            target_angle = self.calculate_angle(current_point, target_point)
            diff_angle = self.calculate_diff_angle(self.yaw, target_angle)
            if diff_distance > DISTANCE_EPSILON * 2.0:
                self.cmd.angular.z = self.calculate_angular_vel(diff_angle, is_moving=True)
            else:
                self.cmd.angular.z = 0.0
            
            # Move robot
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()
            # Recalcualte distance
            current_point = self.get_current_position()
            d += self.calculate_distance(last_position, current_point)
            diff_distance = self.calculate_distance(current_point, target_point)
            last_position = current_point
        
        self.stop_robot()
        self.print_diff_points(current_point, target_point)

        
    def print_diff_points(self, current_point: Point, target_point: Point):
        print("  - Current: (", current_point.x, ",", current_point.y, ")")
        print("  - Target: (", target_point.x, ",", target_point.y, ")")
        print("  - Diff(x,y): ", target_point.x - current_point.x, target_point.y - current_point.y)
        print("  - Distance: ", self.calculate_distance(current_point, target_point))
        
    def arrived_point(self, point: Point):
        current_point = self.get_current_position()
        distance = self.calculate_distance(current_point, point)
        if distance < 1.0:
            return True
        return False

    def start(self):
        rospy.wait_for_message("/odom2", Odometry)
        if not self.is_bag:
            rospy.wait_for_message("/is_pause", Bool)

        zero_x = self.routes[0]["x"]
        zero_y = self.routes[0]["y"]

        for i in range(len(self.routes)):
            node = self.routes[i]
            point = Point(node["x"] - zero_x, node["y"] - zero_y, 0)
            if self.is_bag:
                while not rospy.is_shutdown() and not self.arrived_point(point):
                    self.rate.sleep()
                    continue
            else:
                self.go_to_point(point)

            self.arrived_publisher.publish(Int16(i))

        self.rotate_to_point(Point(1, 0, 0))

if __name__ == "__main__":
    robot = HuskyRobot()
    robot.start()
