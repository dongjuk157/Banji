import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import e, pi, cos, sin, sqrt, atan2, atan
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_lidar = False
        self.collision = False

        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.robot_yaw = 0.0

        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0

    def timer_callback(self):

        if self.is_status and self.is_odom == True and self.is_lidar == True:
            # x = 0
            # y = 0
            # xbar = 179.0
            # xbar /= 2.0
            # ybar = 0.0
            # Sxx = 0.0
            # Sxy = 0.0
            # xl = 85
            # xr = 96
            # firstY = 0

            # left = 0
            # for i in range(xl, xr):
            #     Sxx += (i - xbar) * (i - xbar)
            # for i in range(xl, xr):
            #     y = self.lidar_msg.ranges[i] * cos(pi * ((i - 90) / 180)) * 30
            #     Sxy += (i - xbar) * y
            #     ybar += y

            # ybar /= (xr - xl)
            # beta = Sxy / Sxx
            # ans = atan(beta) / pi * 180
            # print(ans)
            left = self.lidar_msg.ranges[75]
            #right = self.lidar_msg.ranges[270+15]
            #mis_value = (0.35 - left) * 1.5
            self.cmd_msg.linear.x = 0.25
            self.cmd_msg.angular.z = 0.0
            # print(left)
            self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        if self.is_odom == True:

            pcd_msg = PointCloud()
            pcd_msg.header.frame_id = 'map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
            t = np.array([[cos(theta), -sin(theta), pose_x],
                         [sin(theta), cos(theta), pose_y],
                          [0, 0, 1]])
            for angle, r in enumerate(msg.ranges):
                global_point = Point32()

                if 0.0 < r < 12:
                    local_x = r*cos(angle*pi/180)
                    local_y = r*sin(angle*pi/180)
                    local_point = np.array([[local_x], [local_y], [1]])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)

            self.collision = False
            for waypoint in self.path_msg.poses:
                for lidar_point in pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x - lidar_point.x,
                                    2) + pow(waypoint.pose.position.y-lidar_point.y, 2))
                    if distance < 0.1:
                        self.collision = True
                        print('collision')

            self.is_lidar = True


def main(args=None):
    rclpy.init(args=args)
    follow = followTheCarrot()
    rclpy.spin(follow)
    follow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
