from socketio import client
import rclpy
from rclpy.node import Node
from sub3.iot_udp import *
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
import time
import os
import socket
import threading
import struct
import binascii
import copy
import numpy as np
import cv2
import base64
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CompressedImage, LaserScan
from sub1.perception import *
import socketio


sio = socketio.Client()
sio.connect('http://localhost:12001/')

class ros_iot_connect(Node):

    def __init__(self):
        super().__init__('ros_iot_connect')

        # iot_udp 객체 생성
        self.iot = iot_udp()
        time.sleep(0.5)
        # envir_status 데이터
        self.envir = {"day": 0, "hour": 0, "minute": 0,
                      "month": 0, "temperature": 0, "weather": ""}
        self.ebvir_sub = self.create_subscription(
            EnviromentStatus, '/envir_status', self.envir_callback, 10)
        
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
        
    def img_callback(self, msg):
        self.timer += 1
        if self.timer < 20:
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


def main(args=None):
    rclpy.init(args=None)
    conMod = ros_iot_connect()
    rclpy.spin(conMod)
    conMod.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
