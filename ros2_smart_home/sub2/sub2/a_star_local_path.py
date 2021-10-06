import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import LaserScan, PointCloud
import numpy as np
from math import pi,cos,sin,sqrt

# a_star_local_path 노드는 a_star 노드에서 나오는 전역경로(/global_path)를 받아서,
# 로봇이 실제 주행하는 지역경로(/local_path)를 publish 하는 노드입니다.
# path_pub 노드와 하는 역할은 비슷하나, path_pub은 텍스트를 읽어서 global_path를 지역경로를 생성하는 반면,
# a_star_local_path는 global_path를 다른 노드(a_star)에서 받아서 지역경로를 생성합니다.

# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. global_path 데이터 수신 후 저장
# 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 4. global_path 중 로봇과 가장 가까운 포인트 계산
# 5. local_path 예외 처리


class astarLocalpath(Node):

    def __init__(self):
        super().__init__('a_star_local_path')
        # 로직 1. publisher, subscriber 만들기
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Path,'/global_path',self.path_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        # self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # 변수 지정
        self.odom_msg = Odometry()
        self.is_odom = False
        self.is_path = False
        self.is_lidar=False
       
        self.global_path_msg = Path()


        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period = 0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 30 
        self.count = 0


    def listener_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg


    def path_callback(self, msg):
        '''
        로직 2. global_path 데이터 수신 후 저장
        '''
        self.is_path = True
        self.global_path_msg = msg

        
    def timer_callback(self):
        if self.is_odom and self.is_path ==True:
            
            local_path_msg = Path()
            local_path_msg.header.frame_id = '/map'
            
            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y
            current_waypoint = -1
            
            '''
            로직 4. global_path 중 로봇과 가장 가까운 포인트 계산
            '''           
            min_dis = float('inf')
            for i, waypoint in enumerate(self.global_path_msg.poses) : 
                distance = sqrt(
                    pow((x - waypoint.pose.position.x), 2) +
                    pow((y - waypoint.pose.position.y), 2)
                )
                if distance < min_dis :
                    min_dis = distance
                    current_waypoint = i
            
            '''
            로직 5. local_path 예외 처리 
            '''           
            if current_waypoint != -1 : 
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    # 가장 가까운 포인트에서 로컬 경로의 사이즈 만큼 전역경로가 남아있는 경우 로컬 경로의 사이즈(20)만큼 추가
                    for idx in range(current_waypoint, current_waypoint + self.local_path_size):                 
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[idx].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[idx].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)

                else :
                    # 가장 가까운 포인트에서 로컬 경로의 사이즈 만큼 전역경로가 남아있지 않는 경우 나머지 경로 추가
                    for idx in range(current_waypoint, len(self.global_path_msg.poses)):                 
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[idx].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[idx].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)  
                # theta = self.robot_yaw
                # t=np.array([
                #             [cos(theta), -sin(theta), x],
                #             [sin(theta), cos(theta), y],
                #             [0, 0, 1]])
                # pcd_msg = PointCloud()
                # pcd_msg.header.frame_id='map' 
                # for angle, r in enumerate(self.lidar_msg.ranges):
                #     global_point = Point32()
                #     if 0.0 < r < 12:
                #         # 극좌표계를 직교좌표계로 만들어주는 부분
                #         local_x = r*cos(angle*pi/100)
                #         local_y = r*sin(angle*pi/100)
                #         local_point = np.array([[local_x], [local_y], [1]])
                #         global_result=t.dot(local_point)
                #         # 로컬의 극좌표들을 글로벌, 직교좌표로 넣어준다
                #         global_point.x=global_result[0][0]
                #         global_point.y=global_result[1][0]
                #         pcd_msg.points.append(global_point)
            self.local_path_pub.publish(local_path_msg)
    
        # 라이다를 통해 충돌 판단 
    # def lidar_callback(self, msg):
    #     self.lidar_msg=msg
    #     # 경로와 위치 
    #     if self.is_path == True and self.is_odom == True:
    #         # 직교좌표를 변환 
    #         pcd_msg = PointCloud()
    #         pcd_msg.header.frame_id='map'

    #         pose_x = self.odom_msg.pose.pose.position.x
    #         pose_y = self.odom_msg.pose.pose.position.y
    #         theta = self.robot_yaw
    #         t=np.array([
    #                     [cos(theta), -sin(theta), pose_x],
    #                     [sin(theta), cos(theta), pose_y],
    #                     [0, 0, 1]])
    #         for angle, r in enumerate(msg.ranges):
    #             global_point = Point32()
    #             if 0.0 < r < 12:
    #                 # 극좌표계를 직교좌표계로 만들어주는 부분
    #                 local_x = r*cos(angle*pi/100)
    #                 local_y = r*sin(angle*pi/100)
    #                 local_point = np.array([[local_x], [local_y], [1]])
    #                 global_result=t.dot(local_point)
    #                 # 로컬의 극좌표들을 글로벌, 직교좌표로 넣어준다
    #                 global_point.x=global_result[0][0]
    #                 global_point.y=global_result[1][0]
    #                 pcd_msg.points.append(global_point)
    #         self.collision = False
    #         # print(self.path_msg.poses)
    #         # 모든 경로점(로컬)과 모든 라이다와의 거리를 계산
    #         for waypoint in self.path_msg.poses[:10]:
    #             for lidar_point in pcd_msg.points:
    #                 distance=sqrt(pow(waypoint.pose.position.x-lidar_point.x, 2)+pow(waypoint.pose.position.y-lidar_point.y, 2))
    #                 if distance < 0.05:
    #                     print(distance)
    #                     self.collision=True
    #         self.is_lidar=True
    #         self.pcd_pub.publish(pcd_msg)

        
def main(args=None):
    rclpy.init(args=args)

    a_star_local = astarLocalpath()

    rclpy.spin(a_star_local)

    a_star_local.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
