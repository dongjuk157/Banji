import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi, cos, sin, sqrt, atan2
import numpy as np

# path_tracking 노드는 로봇의 위치(/odom), 로봇의 속도(/turtlebot_status), 주행 경로(/local_path)를 받아서, 주어진 경로를 따라가게 하는 제어 입력값(/cmd_vel)을 계산합니다.
# 제어입력값은 선속도와 각속도로 두가지를 구합니다.
# sub2의 path_tracking은 sub1의 path_tracking를 사용해도 됩니다.


# 노드 로직 순서
# 1. 제어 주기 및 타이머 설정
# 2. 파라미터 설정
# 3. Quaternion 을 euler angle 로 변환
# 4. 터틀봇이 주어진 경로점과 떨어진 거리(lateral_error)와 터틀봇의 선속도를 이용해 전방주시거리 계산
# 5. 전방 주시 포인트 설정
# 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산
# 7. 선속도, 각속도 정하기


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/local_path', self.path_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.pcd_pub = self.create_publisher(PointCloud, 'pcd', 10)
        self.pcd_sub = self.create_subscription(
            PointCloud, '/pcd', self.pcd_callback, 10)

        # 로직 1. 제어 주기 및 타이머 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_lidar = False
        self.collision = False
        self.is_pcd = False
        self.odom_msg = Odometry()
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.pcd_msg = PointCloud()
        self.collision_cnt = 0

        # 로직 2. 파라미터 설정
        # lfd: 전방주시거리 및 최소, 최대 전방주시거리를 설정
        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0

    def timer_callback(self):
        # 계산에 필요한 데이터들이 들어 왔는지 확인
        if self.is_status and self.is_odom == True and self.is_path == True:

            # 경로점이 1개 초과인 경우
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False

                # 로봇의 현재 위치를 나타내는 변수
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y

                # 로봇이 경로에서 떨어진 거리를 나타내는 변수
                # local_path가 있으면 로봇과 가장 가까이 있는 경로점과 로봇과의 거리를 계산
                # 왜 이걸 계산하면 멀리떨어져있을 때 전방주시거리를 멀리보게하기 위해서 구한다.
                # 피타고라스 !
                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x, 2)+pow(
                    self.path_msg.poses[0].pose.position.y-robot_pose_y, 2))
                # print(robot_pose_x,robot_pose_y,lateral_error)
                '''
                로직 4. 로봇이 주어진 경로점과 떨어진 거리(lateral_error)와 로봇의 선속도를 이용해 전방주시거리 설정
                
                self.lfd= 
                
                if self.lfd < self.min_lfd :
                    self.lfd=self.min_lfd
                if self.lfd > self.max_lfd:
                    self.lfd=self.max_lfd

                '''
                # lateral_error와 주행하는 선속도와 적절하게 전방주시거리를 설정한다.
                # 너무 작거나 커질 수 있기에 최소 최대로 한정짓는다. 사전영상 18분정도쯤 참고 !

                self.lfd = (self.status_msg.twist.linear.x+lateral_error)*0.5

                if self.lfd < self.min_lfd:
                    self.lfd = self.min_lfd

                if self.lfd > self.max_lfd:
                    self.lfd = self.max_lfd

                min_dis = float('inf')
                '''
                로직 5. 전방 주시 포인트 설정
                for num,waypoint in enumerate(self.path_msg.poses) :

                    self.current_point=
                    dis=
                    if abs(dis-self.lfd) < min_dis :
                        min_dis=
                        self.forward_point=
                        self.is_look_forward_point=

                '''
                for num, waypoint in enumerate(self.path_msg.poses):
                    # 현재 경로점
                    self.current_point = waypoint.pose.position
                    # lfd만큼 떨어져 있는 경로점을 찾는 부분임 (피타고라스 적용, 로봇과 경로점 간)
                    dis = sqrt(pow(self.path_msg.poses[0].pose.position.x-self.current_point.x, 2) +
                               pow(self.path_msg.poses[0].pose.position.y-self.current_point.y, 2))
                    if abs(dis-self.lfd) < min_dis:
                        min_dis = abs(dis-self.lfd)
                        self.forward_point = self.current_point
                        # 전방주시거리만큼 떨어진 경로점을 찾았다
                        self.is_look_forward_point = True

                if self.is_look_forward_point:

                    global_forward_point = [
                        self.forward_point.x, self.forward_point.y, 1]

                    '''
                    로직 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산

                    (테스트) 맵에서 로봇의 위치(robot_pose_x,robot_pose_y)가 (5,5)이고, 헤딩(self.robot_yaw) 1.57 rad 일 때, 선택한 전방포인트(global_forward_point)가 (3,7)일 때
                    변환행렬을 구해서 전방포인트를 로봇 기준좌표계로 변환을 하면 local_forward_point가 구해지고, atan2를 이용해 선택한 점과의 각도를 구하면
                    theta는 0.7853 rad 이 나옵니다.
                    trans_matrix는 로봇좌표계에서 기준좌표계(Map)로 좌표변환을 하기위한 변환 행렬입니다.
                    det_tran_matrix는 trans_matrix의 역행렬로, 기준좌표계(Map)에서 로봇좌표계로 좌표변환을 하기위한 변환 행렬입니다.  
                    local_forward_point 는 global_forward_point를 로봇좌표계로 옮겨온 결과를 저장하는 변수입니다.
                    theta는 로봇과 전방 주시 포인트와의 각도입니다. 

                    trans_matrix=
                    det_trans_matrix=
                    local_forward_point=
                    theta=
                    
                    '''
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -
                         sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(
                            self.robot_yaw), robot_pose_y],
                        [0, 0, 1]])
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(
                        global_forward_point)
                    # lfd를 더한 경로점과 로봇과의 사이 각 / 추후 내용 더 작성예정
                    theta = - \
                        atan2(local_forward_point[1], local_forward_point[0])

                    '''
                    로직 7. 선속도, 각속도 정하기
                    out_vel=
                    out_rad_vel=

                    '''

                    # 적절하게 주어지게한다
                    out_vel = 0.6
                    # 앞서 구한 theta를 이용해 제어할 각속도로 이용한다 / 크면 클수록 경로에 빠르게 수렴함. 천천히는 반대
                    out_rad_vel = theta
                    # print(theta)
                    self.cmd_msg.linear.x = out_vel
                    self.cmd_msg.angular.z = out_rad_vel

                    if self.collision == True:
                        print('collision')
                        if self.is_pcd == True:
                            length = len(self.pcd_msg.points)
                            # 라이다
                            back_dist = sqrt(pow(robot_pose_x-self.pcd_msg.points[length//2-1].x, 2)+pow(
                                robot_pose_y-self.pcd_msg.points[length//2-1].y, 2))

                        # 충돌 날 시 후진
                        self.cmd_msg.linear.x = -0.5
                        if theta < 0:
                            self.cmd_msg.angular.z = -0.4
                        else:
                            self.cmd_msg.angular.z = 0.4

                        # # 뒤에서 충돌날 시 => 잘 안먹음
                        # if back_dist < 0.01:
                        #     print('후방', back_dist)
                        #     self.cmd_msg.linear.x=0.5
                        #     self.cmd_msg.angular.z=0.2

            else:
                print("no found forward point")
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                # left=self.lidar_msg.ranges[75]
                # right=self.lidar_msg.ranges[270+15]
                # mis_value=(right-left)*1.3
                # self.cmd_msg.linear.x=0.5
                # self.cmd_msg.angular.z=mis_value

            self.cmd_pub.publish(self.cmd_msg)

    # 라이다를 통해 충돌 판단
    def lidar_callback(self, msg):
        self.lidar_msg = msg
        # 경로와 위치
        if self.is_path == True and self.is_odom == True:
            # 직교좌표를 변환
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id = 'laser'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw

            t = np.array([
                [cos(theta), -sin(theta), pose_x],
                [sin(theta), cos(theta), pose_y],
                [0, 0, 1]])

            for angle, r in enumerate(msg.ranges):
                global_point = Point32()
                if 0.0 < r < 12:
                    # 극좌표계를 직교좌표계로 만들어주는 부분
                    local_x = r*cos(angle*pi/100)
                    local_y = r*sin(angle*pi/100)
                    local_point = np.array([[local_x], [local_y], [1]])
                    global_result = t.dot(local_point)
                    # 로컬의 극좌표들을 글로벌, 직교좌표로 넣어준다
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    global_point.z = 1.0
                    pcd_msg.points.append(global_point)
            # print(len(pcd_msg.points))
            self.collision = False
            # print(self.path_msg.poses)
            # 모든 경로점(로컬)과 모든 라이다와의 거리를 계산
            for waypoint in self.path_msg.poses[:10]:
                # print(waypoint)
                for lidar_point in pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x-lidar_point.x,
                                    2)+pow(waypoint.pose.position.y-lidar_point.y, 2))
                    if distance < 0.01:
                        # print(distance)
                        self.collision = True
            self.is_lidar = True
            self.pcd_pub.publish(pcd_msg)

    def pcd_callback(self, msg):
        self.is_pcd = True
        self.pcd_msg = msg

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        '''
        로직 3. Quaternion 을 euler angle 로 변환
        q=
        _,_,self.robot_yaw=

        '''
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg


def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
