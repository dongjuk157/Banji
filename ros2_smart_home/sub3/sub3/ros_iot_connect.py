import rclpy
from rclpy.node import Node
from sub3.iot_udp import *
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
import time
import threading
import numpy as np
import cv2
import base64
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CompressedImage
from sub1.perception import *
import socketio


sio = socketio.Client()
sio.connect('http://j5b301.p.ssafy.io:12001')


class ros_iot_connect(Node):

    def __init__(self):
        super().__init__('ros_iot_connect')
        print("ros연결")
        # iot_udp 객체 생성
        self.iot = iot_udp()
        self.pos = [0., 0.]
        time.sleep(0.5)
        # envir_status 데이터
        self.envir = {"day": 0, "hour": 0, "minute": 0,
                    "month": 0, "temperature": 0, "weather": ""}
        self.ebvir_sub = self.create_subscription(
            EnviromentStatus, '/envir_status', self.envir_callback, 10)

        self.goal_pose_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)

        self.subscription = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.listener_callback, 10)

        # self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.cmd_msg = Twist()


        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        self.turtle_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status',self.turtlebot_status_callback,1)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.battery = None
        self.power = None
        ###########################################################################
        # image
        self.screenshot = ''
        self.robotView = ''
        self.timer = 0
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        ###########################################################################

        self.scenario_object = {}
        # 스캔된 데이터 목록
        self.scanned_reg_objs = {}
        self.scanned_new_objs = {}

        # mutex lock
        self.lock = threading.Lock()

    def envir_callback(self, msg):
        self.envir = msg

    # 수동 주행
    def turtlebot_status_callback(self, msg):
        # global linear_x,angular_z
        self.battery = msg.battery_percentage
        self.power = msg.power_supply_status
        self.linear_x = msg.twist.linear.x
        self.angular_z = msg.twist.angular.z
    
    def robot_movement(self,move_cmd):

        # global linear_x,angular_z
        # print("11",linear_x,angular_z)
        linear_x = self.linear_x
        angular_z = self.angular_z

        if move_cmd == 'left':

            self.cmd_msg.linear.x=0.0
            self.cmd_msg.angular.z=-0.2
            # while angular_z > -0.1:
                # self.cmd_publisher.publish(self.cmd_msg)

        elif move_cmd == 'go':
            
            self.cmd_msg.linear.x= 0.2
            self.cmd_msg.angular.z= 0.0
            # while linear_x < 0.1:
                # self.cmd_publisher.publish(self.cmd_msg)

        elif move_cmd == 'back':
            
            self.cmd_msg.linear.x=-0.2
            self.cmd_msg.angular.z=0.0
            # while linear_x > -0.1:
                # self.cmd_publisher.publish(self.cmd_msg)

        elif move_cmd == 'right':
            
            self.cmd_msg.linear.x=0.0
            self.cmd_msg.angular.z=0.2
            # while angular_z < 0.1:
                # self.cmd_publisher.publish(self.cmd_msg)
        
        else:

            self.cmd_msg.linear.x=0.0
            self.cmd_msg.angular.z=0.0
            # while (linear_x > 0.01 or linear_x < -0.01) or (angular_z > 0.01 or angular_z< -0.01):
        # self.cmd_publisher.publish(self.cmd_msg)
        self.cnt = 7
        while self.cnt > 0 and ((abs(linear_x - self.cmd_msg.linear.x) > 0.1) or (abs(angular_z - self.cmd_msg.angular.z) > 0.1)):
            self.cnt -= 1
            time.sleep(0.3)
            self.cmd_publisher.publish(self.cmd_msg)
        
    ##########################################################

    def img_callback(self, msg):
        self.timer += 1
        if self.timer < 15:
            return
        self.timer = 0
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        screenshot_path = 'C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\ros2_smart_home\\sub3\\sub3\\images\\screenshot.png'
        robotview_path = 'C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\ros2_smart_home\\sub3\\sub3\\images\\robotview.png'
        cv2.imwrite(screenshot_path, img_bgr)
        with open(screenshot_path, 'rb') as screenshot_img:
            self.screenshot = base64.b64encode(screenshot_img.read())

        cv2.imwrite(robotview_path, img_bgr)
        with open(robotview_path, 'rb') as robotview_img:
            self.robotView = base64.b64encode(robotview_img.read())
            sio.emit('back_robotview_robot', self.robotView.decode('utf-8'))

    def gotothere(self, data):
        goal_pose = PoseStamped()
        print(data)
        goal_pose.header.frame_id = 'map'
        x = (data[0]+(1/0.05)*-14.75)/20
        y = (data[1]+(1/0.05)*2.25)/20
        print(x, y)
        goal_pose.pose.position.x = x  # -14.367 -4.851
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        print(goal_pose)
        self.goal_pose_pub.publish(goal_pose)

    def listener_callback(self, msg):
        self.pos[0] = msg.twist.angular.x
        self.pos[1] = msg.twist.angular.y

    def loadmap(self):
        full_path = 'C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\ros2_smart_home\\sub3\\map\\map.txt'
        self.f = open(full_path, 'r')
        map_size_x = 350
        map_size_y = 350
        map_data = [0 for i in range(map_size_x*map_size_y)]
        line = self.f.readline()
        line_data = line.split()
        for num, data in enumerate(line_data):
            map_data[num] = int(data)
        map_to_grid = np.array(map_data)
        grid = np.reshape(map_to_grid, (350, 350))

        for y in range(350):
            for x in range(350):
                if grid[x][y] == 100:
                    for i in range(-5, 6):
                        for j in range(-5, 6):
                            if 0 <= x+i < 350 and 0 <= y+j < 350 and grid[x+i][y+j] < 80:
                                grid[x+i][y+j] = 127

        np_map_data = grid.reshape(1, 350*350)
        np_map_data = np_map_data
        list_map_data = np_map_data.tolist()

        # 로직2를 완성하고 주석을 해제 시켜주세요.
        self.f.close()
        print('read_complete')
        return list_map_data[0]


def main(args=None):
    rclpy.init(args=None)
    conMod = ros_iot_connect()
    rclpy.spin(conMod)
    conMod.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
