import rclpy
from rclpy.node import Node
from sub3.ros_iot_connect import *
from sub3.iot_udp import *
import socketio
import threading
import time
from sub2.human_detector import *
import base64


# client 는 socketio의 기본 API로 구성된 노드입니다. 서버와 연결을 시도해서 서버와 통신을 합니다.

# 각자의 서버 주소에 맞게 connect 함수 안을 바꿔주고, server 스켈레톤코드를 이용해 서비스를 하고 있다면, 연결이 됩니다.
# 버튼을 누르면 해당 키값에 맞는 함수들이 호출이 됩니다. 연결이 된 후에는 emit 함수를 이용해 서버로 키값과 데이터를 보냅니다.
# 이 노드는 AWS EC2에 구축한 서버와 통신만 하는 노드이고, ROS2와 연동하여 사용하면 스마트홈에서 얻은 데이터들을 서버로 보내고, 웹서버로부터의 명령을 ROS2로 전달할 수 있습니다.

# 노드 로직 순서
# 1. 클라이언트 소켓 생성
# 2. 데이터 수신 콜백함수
# 3. 서버 연결
# 4. 데이터 송신


def startConMod(conMod):
    rclpy.spin(conMod)
    conMod.destroy_node()
    rclpy.shutdown()


try:
    # 로직 1. 클라이언트 소켓 생성
    rclpy.init(args=None)
    conMod = ros_iot_connect()
    sio = socketio.Client()

    @sio.event
    def connect():
        conMod_thread = threading.Thread(target=startConMod, args=[conMod])
        conMod_thread.daemon = True
        conMod_thread.start()
        print('connection established')

    # 로직 2. 데이터 수신 콜백함수
    # 프런트가 로봇을 움직인 방법

    @sio.on('robot_loadmap_back')
    def loadmap(data):
        sio.emit('back_loadmap_robot', conMod.loadmap())

    @sio.on('robot_move_back')
    def goTothere(data):
        conMod.gotothere(data)
        print("이동")
        # conMod.move()

    @sio.on('robot_position_back')
    def position(data):
        x = int((conMod.pos[0] + 14.75) / 0.05)
        y = int((conMod.pos[1] - 2.25) / 0.05)
        mes = [x, y]
        sio.emit('back_position_robot', mes)
    # time/weather/temperature refresh
    # 프런트 보낼 때방법

    @sio.on("robot_robotStatus_back")
    def robot_status(data):
        # this.status.battery = message.battery;
        # this.status.power = message.power;
        message = {
            'battery': conMod.battery,
            'power': conMod.power,
        }
        sio.emit('back_robotStatus_robot', message)

    @sio.on("robot_environment_back")
    def robot_environment(data):
        message = {
            'month': conMod.envir.month,
            'day': conMod.envir.day,
            'hour': conMod.envir.hour,
            'minute': conMod.envir.minute,
            'temperature': conMod.envir.temperature,
            'weather': conMod.envir.weather,
        }
        sio.emit('back_environment_robot', message)

    @sio.on("robot_control_back")
    def robot_control(data):
        print(data)
        # data = { (int)index, (str)name, (str)status, (str)control }
        # 1, '에어컨', 'OFF', 'ON'
        # status: front가 인식한 상태
        # control: 바꾸고 싶은 상태
        # 껐다고 판단해도 제대로 안꺼진 경우가 있을수 있어서 control이 필요하다고 생각함

        # 1. idx, name에 맞는 가전 위치 찾기

        # 2. 해당 위치로 이동

        # 3. 가전 제어
        conMod.iot.connect()
        conMod.iot.control()  # 현재 toggle 밖에 안됨
        conMod.iot.disconnect()

        # 4. 상태 갱신
        status = 'ON' if data['status'] == 'OFF' else 'OFF'  # 테스트용 토글

        # 5. 완료 메시지 송신
        message = {
            'index': data['index'],
            'name': data['name'],
            'status': status,
        }
        print("EMIT!!!")
        sio.emit('back_control_robot', message)

    @sio.on("robot_movemanualy_back")
    def move_manually(data):
        print("come", data)
        conMod.robot_movement(data)
        # print("gone",data)

    # 스크린샷 보내기
    @sio.on("robot_screenshot_back")
    def send_screenshot(data):
        message = conMod.screenshot
        sio.emit('back_screenshot_robot', message.decode('utf-8'))

    @sio.event
    def disconnect():
        print('disconnected from server')

    # 로직 3. 서버 연결
    sio.connect('http://localhost:12001/')

    # 로직 4. 데이터 송신
    sio.emit('sendTime', 'TEST')

    sio.wait()

except KeyboardInterrupt:
    exit()
