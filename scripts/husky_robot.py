#!/usr/bin/env python
from turtle import distance
from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import Header
from genpy import Time
from geometry_msgs.msg import Twist, Point, Vector3
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
from geodesy import utm
from math import atan2, degrees, pi, sqrt

DISTANCE_EPSILON = 1e-4
ANGULAR_EPSILON = degrees(2)

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.3

class HuskyRobot():
    def __init__(self) -> None:
        rospy.init_node('husky_robot_control', anonymous=True)
        # Publishers
        self.vel_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.gps_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

        rospy.on_shutdown(self.shutdown_hook)

        # Vars
        self.rate = rospy.Rate(10)
        self.ctrl_c = False
        self.is_pause = True

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0
        # Orientation
        self.yaw = 0.0
        self.gps_yaw = 0.0
        # GPS
        self.lat = 0.0
        self.long = 0.0
        self.alt = 0.0
        # Velocity
        self.imu_time = Time()
        self.angular_velocity = Vector3()

    # Callbacks
    def shutdown_hook(self):
        self.ctrl_c = True
    
    def imu_callback(self, msg: Imu):
        self.imu_time = msg.header.stamp
        self.angular_velocity = msg.angular_velocity

        orientation_q = msg.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def gps_callback(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude

    def update_gps_yaw(self, p1: Point, p2: Point):
        self.gps_yaw = self.calculate_angle(p1, p2)

    # Helpers
    def is_running(self) -> Boolean:
        return not self.ctrl_c

    def get_current_position(self) -> Point:
        point = utm.fromLatLong(self.lat, self.long).toPoint()
        return point

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

    def get_time(self, t:Time):
        return t.secs + float(t.nsecs) / 1e9

    # Control robots
    def stop_robot(self) -> None:
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_publisher.publish(self.cmd)
        self.rate.sleep()

    def go_straight(self, distance, update_yaw=False) -> None:
        last_point = self.get_current_position()
        d = 0.0

        while d < abs(distance) and not self.ctrl_c:
            if distance < 0:
                self.cmd.linear.x = -LINEAR_SPEED
            else:
                self.cmd.linear.x = LINEAR_SPEED
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            current_point = self.get_current_position()
            d += self.calculate_distance(last_point, current_point)
            if update_yaw:
                self.update_gps_yaw(last_point, current_point)

            last_point = current_point
        # Stop robot
        self.stop_robot()

    def relocate(self):
        if self.is_running():
            self.rate.sleep()
            start_point = self.get_current_position()
            self.go_straight(0.5)
            end_point = self.get_current_position()
            self.update_gps_yaw(start_point, end_point)
            self.go_straight(-0.5)

    # Rotate by IMU data
    def rotate(self, angle):
        print("  - Rotate ", angle)
        rotated_angle = 0.0
        last_t = self.get_time(self.imu_time)
        self.cmd.angular.z = ANGULAR_SPEED if angle < 0 else -ANGULAR_SPEED

        while rotated_angle < abs(angle) and self.is_running():
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

            current_t = self.get_time(self.imu_time)
            delta_t = current_t - last_t
            rotated_angle += delta_t * abs(self.angular_velocity.z)
            last_t = current_t
            # print(delta_t, abs(self.angular_velocity.z), rotated_angle)
            
        self.stop_robot()


    def go_to_point(self, target_point: Point):
        current_point = self.get_current_position()
        self.print_diff_points(current_point, target_point)

        target_angle = self.calculate_angle(self.get_current_position(), target_point)
        diff_angle = self.calculate_diff_angle(self.gps_yaw, target_angle)
        self.rotate(diff_angle)

        diff_distance = self.calculate_distance(self.get_current_position(), target_point)
        max_distance = 1.1 * diff_distance
        last_position = self.get_current_position()

        print("  - Distance to next point: ", diff_distance)
        
        d = 0.0
        while d < max_distance and diff_distance > DISTANCE_EPSILON and self.is_running():
            # Calculate linear vel
            self.cmd.linear.x = LINEAR_SPEED
            # Calculate angular vel
            target_angle = self.calculate_angle(self.get_current_position(), target_point)
            diff_angle = self.calculate_diff_angle(self.gps_yaw, target_angle)
            if abs(diff_angle) > ANGULAR_EPSILON:
                self.cmd.angular.z = ANGULAR_SPEED if diff_angle > 0 else -ANGULAR_SPEED
            else:
                self.cmd.angular.z = 0.0
            # Move robot
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()
            # Recalcualte distance
            current_point = self.get_current_position()
            self.update_gps_yaw(last_position, current_point)
            d += self.calculate_distance(last_position, current_point)
            diff_distance = self.calculate_distance(current_point, target_point)
            last_position = current_point
        
        self.stop_robot()
        self.print_diff_points(current_point, target_point)

        
    def print_diff_points(self, current_point: Point, target_point: Point):
        print("  Current: (", current_point.x, ",", current_point.y, ")")
        print("  Target: (", target_point.x, ",", target_point.y, ")")
        print("  Diff: ", target_point.x - current_point.x, target_point.y - current_point.y)
        print("  Distance: ", self.calculate_distance(current_point, target_point))

    